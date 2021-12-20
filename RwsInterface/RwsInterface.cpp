/*
interface.h
This class is for interfacing RobWorkStudio, C++ programs, and OpenCV.
Author: Arthicha Srisuchinnawong
Email: arsri21@student.sdu.dk
*/

/******************************************************************************
*                             Include Libraries                               *
******************************************************************************/

// RobWork libraries
#include "RwsInterface.h"

/*****************************************************************************
*                            Setup and Initialization                        *
*****************************************************************************/

RwsInterface::RwsInterface(const string &_wc_file)
{
    /* 
    function type: constructor
    input: relative path to Scene (e.g., Scene.wc.xml)
    output: none
    detail: initialize robwork studio interface
    */
    this->wc_file = _wc_file;
    this->wc = WorkCellLoader::Factory::load (_wc_file);
    this->ur = this->wc->findDevice<SerialDevice>("UR-6-85-5-A");
    this->gripper = this->wc->findDevice<TreeDevice>("WSG50");
    this->cams.push_back(this->wc->findFrame ("Camera_Right"));
    this->cams.push_back(this->wc->findFrame ("Camera_Left"));
    this->cams.push_back(this->wc->findFrame ("Scanner25D"));

    this->state = this->wc->getDefaultState();

    for(int i =0;i<3;i++) // camera parameter inilization
    {
        this->K.push_back(Mat::zeros(3,3,CV_64F));
        this->A.push_back(Mat::zeros(3,4,CV_64F));
        this->H.push_back(Mat::zeros(4,4,CV_64F));
    }
}

RwsInterface::~RwsInterface()
{
    /* 
    function type: destrictor
    input: none
    output: none
    detail: none
    */
}

void RwsInterface::setup(RobWorkStudioApp* _app)
{
    /* 
    function type: public setup function
    input: robwork studio app
    output: none
    detail: link robwork studio app to robwork studio interface
            Note that _app should be created using RobWorkStudioApp app ("");
            and follow this format
            1. RwsInterface sim(WC_FILE); // initialize robwork studio interface
            2. RobWorkStudioApp app (""); // initialize robwork studio app
            3. RWS_START (app) // start robwork studio (as a function)
            4. {
            5.  sim.setup(&app); // setup (this function)
            6.  //do something
            7. }RWS_END() // close the robwork studio
    */
    this->app = _app;
    this->rwstudio = this->app->getRobWorkStudio ();
    this->rwstudio->postOpenWorkCell(this->wc_file);
    this->app->getRobWorkStudio()->setWorkCell(this->wc);
    this->tcp = this->wc->findFrame("UR-6-85-5-A.TCP");
    this->tool = this->wc->findFrame("WSG50.TCP");
    this->urbase = this->wc->findFrame<MovableFrame>("URReference");
    this->urref = this->wc->findFrame<MovableFrame>("URReference");
    this->goal = this->wc->findFrame<MovableFrame>("Goal");
    this->objectName.push_back("Bottle");
    this->objectName.push_back("Square");
    this->objectName.push_back("Cylinder");
    for(int i =0;i<this->objectName.size(); i++){this->object.push_back(this->wc->findFrame<MovableFrame>(this->objectName.at(i)));}
    this->closedFormSovler = ownedPtr( new ClosedFormIKSolverUR(this->ur, this->state) );
    this->detector = new CollisionDetector(this->wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
}

void RwsInterface::setDelay(int t)
{
    /* 
    function type: public setup function
    input: expected delay time in milliseconds
    output: none
    detail: set up the delay time for each iteration
    */
    this->t_delay = t;
}

void RwsInterface::setTargetIdx(int idx)
{
    /* 
    function type: public setup function
    input: target object id
    output: none
    detail: set up the target object id (avoid collision with the target object)
    */
    this->objectId = idx;
}





/*****************************************************************************
*                                 Camera Interface                           *
*****************************************************************************/

Mat RwsInterface::getImage(int camNum)
{
    /* 
    function type: public function
    input: camera number
    output: a matrix of type "Mat" representing the image from specified camera number 
    detail: get image from a specific camera
    */
    const PropertyMap& properties = this->cams.at(camNum)->getPropertyMap ();
    string parameters;
    if(camNum != 2)
    {
        parameters = properties.get<string>("Camera");
    }
    else
    {
        parameters = properties.get<string>("Scanner25D");
    }
     
    istringstream iss (parameters, istringstream::in);
    double fovy;
    int width;
    int height;
    iss >> fovy >> width >> height;
    double f = height / 2 / tan(fovy * (2*M_PI) / 360.0 / 2.0 );
    // update K matrix
    this->K.at(camNum).at<double>(0,0) = width;
    this->K.at(camNum).at<double>(1,1) = height;
    this->K.at(camNum).at<double>(0,2) = width/2.0;
    this->K.at(camNum).at<double>(1,2) = height/2.0;
    this->K.at(camNum).at<double>(2,2) = 1.0;
    // update A matrix
    this->A.at(camNum).at<double>(0,0) = f;
    this->A.at(camNum).at<double>(1,1) = f;
    this->A.at(camNum).at<double>(2,2) = 1.0;
    // update H matrix
    this->H.at(camNum) = this->rwH2cvH(Kinematics::frameTframe(this->wc->getWorldFrame(), this->cams.at(camNum), this->state));
    
    if(camNum == 2)
    {
        auto framegrabber25D = new rwlibs::simulation::GLFrameGrabber25D(width,height,fovy);
        rw::graphics::SceneViewer::Ptr gldrawer = this->rwstudio->getView ()->getSceneViewer();
        framegrabber25D->init(gldrawer);
        framegrabber25D->grab(this->cams.at(camNum), this->state);
        const rw::geometry::PointCloud* img = &(framegrabber25D->getImage());
        int counter = 0;
        for(const auto &p_tmp : img->getData())
        {
            rw::math::Vector3D<float> p = p_tmp;
            if(p(2) > -1.38 && p(1) > -0.48)
            {
                counter++;
            }
        }
        std::ofstream output("scanner25D.pcd");
        output << "# .PCD v.5 - Point Cloud Data file format\n";
        output << "FIELDS x y z\n";
        output << "SIZE 4 4 4\n";
        output << "TYPE F F F\n";
        output << "WIDTH " << (img->getWidth()/8 - 4) << "\n";
        output << "HEIGHT " << (img->getHeight()/8 - 4) << "\n";
        output << "POINTS " << ((img->getWidth()/8 - 4)*(img->getHeight()/8 - 4)) << "\n";
        output << "DATA ascii\n";
        for(const auto &p_tmp : img->getData())
        {
            rw::math::Vector3D<float> p = p_tmp;
            if(p(2) > -1.38 && p(1) > -0.28)
            {
                output << p(0) << " " << p(1) << " " << p(2) << "\n";
            }
        }
        
        for(int i=1; i<=((img->getWidth()/8 - 4)*(img->getHeight()/8 - 4) - counter); i++)
        {
            output << 0 << " " << 0 << " " << 0 << "\n";
        }/*
        std::ofstream output("scanner25D.pcd");
        output << "# .PCD v.5 - Point Cloud Data file format\n";
        output << "FIELDS x y z\n";
        output << "SIZE 4 4 4\n";
        output << "TYPE F F F\n";
        output << "WIDTH " << img->getWidth() << "\n";
        output << "HEIGHT " << img->getHeight() << "\n";
        output << "POINTS " << img->getData().size() << "\n";
        output << "DATA ascii\n";
        for(const auto &p_tmp : img->getData())
        {
            rw::math::Vector3D<float> p = p_tmp;
            output << p(0) << " " << p(1) << " " << p(2) << "\n";
        }*/
        output.close();
    }

        
    Mat result;
    const SceneViewer::Ptr gldrawer = this->rwstudio->getView ()->getSceneViewer();
    const GLFrameGrabber::Ptr framegrabber = ownedPtr (new GLFrameGrabber (width, height, fovy));
    framegrabber->init (gldrawer);
    framegrabber->grab(this->cams.at(camNum), this->state);
    Image* rw_image = &(framegrabber->getImage());
    Mat img(rw_image->getHeight(),rw_image->getWidth(), CV_8UC3);
    img.data = (uchar*)rw_image->getImageData();
    cvtColor( img, result, COLOR_RGB2BGR );

    
    return result;
}

/**************************************************************************
*                            Object Manipulation                          *
**************************************************************************/
void RwsInterface::moveObject(int idx,  float x, float y, float z,float roll, float pitch, float yaw)
{
    /* 
    function type: public function
    input: object id, target position (x, y, z), and target orientation (roll,pitch,yaw) 
    output: none
    detail: move the object to the target position and orientaiton
    */
    Transform3D<> target(Vector3D<>(x,y,z),Rotation3D<>(RPY<>(roll,pitch,yaw)));
    this->object.at(idx)->moveTo(target,this->state);  
}

void RwsInterface::moveUR(float x, float y, float z,float roll, float pitch, float yaw)
{
    /* 
    function type: public function
    input: object id, target position (x, y, z), and target orientation (roll,pitch,yaw) 
    output: none
    detail: move the robot to the target position and orientaiton
    */
    Transform3D<> target(Vector3D<>(x,y,z),Rotation3D<>(RPY<>(roll,pitch,yaw)));
    this->urbase->moveTo(target,this->state);  
    this->urref->moveTo(target,this->state);  
}

void RwsInterface::terminateObject()
{
    /* 
    function type: public function
    input: none
    output: none
    detail: set the target object to be terminated, the object will be move to current pose when the simulation updates
    */
    this->objectTermination = true;
    this->objectTerminatePose = Kinematics::frameTframe(this->wc->getWorldFrame(), this->object[this->objectId], this->state);
}

/**************************************************************************
*                          Kinematics Operation                           *
**************************************************************************/
Transform3D<> RwsInterface::getObjectPose(int idx)
{
    /* 
    function type: public function
    input: object id
    output: the homogeneous matrix of a specific object
    detail: get the homogeneous matrix of a specific object
    */
    return Kinematics::frameTframe(this->urbase, this->object[idx], this->state);
}

Transform3D<> RwsInterface::getTCP()
{
    return Kinematics::frameTframe(this->urbase, this->tool, this->state);
}

Transform3D<> RwsInterface::getFK(Q q)
{
    /* 
    function type: public function
    input: robot confiuration (Q) 
    output: homogeneous transformation matrix of the tool frame
    detail: calculate the forward kinematics of the tool frame
    */
    Q q_pre = this->ur->getQ(this->state);
    this->ur->setQ(q,this->state);
    Transform3D<> frameBaseTGoal = Kinematics::frameTframe(this->urbase, this->tool, this->state);
    this->ur->setQ(q_pre,this->state);
    return frameBaseTGoal;
}

Q RwsInterface::getIK(Transform3D<> frameBaseTGoal)
{
    /* 
    function type: public function
    input: the homogeneous transformation matrix of the target/object/via point 
    output: the corresponding robot configuration (Q)
    detail: calculate the inverse kinematics of the robot
    */

    // preparation
    Q q_pre = this->ur->getQ(this->state);

    // solve IK numerically
    vector<Q> qs = this->closedFormSovler->solve(frameBaseTGoal, this->state);

    // soluton selection
    Q q_best(6,-10.0,-10.0,-10.0,-10.0,-10.0,-10.0);
    float dist_best = 1e10;
    float diff = 0.0;
    cout << "size   "<< qs.size() << endl;
    for (uint i=0;i<qs.size();i++)
    {
        if(not this->checkUrCollision(qs.at(i))) // collision test
        {
            if(this->checkFKIK(frameBaseTGoal,qs.at(i),0.5)) // accuracy test
            {
                diff = (qs.at(i)-q_pre).norm2();
                if (diff < dist_best) // choose minimum moving distance
                {
                    dist_best = diff;
                    q_best = qs.at(i);
                }
            }
        }
    }
    this->ur->setQ(q_pre,this->state);
    return q_best;
}

bool RwsInterface::setFK(Q joint_angle)
{
    /* 
    function type: public function
    input: set of joint angles in configuration space (Q)
    output: none
    detail: move the robot to a specific configuration according to the input
    */
    this->ur->setQ(joint_angle,this->state);
    return this->checkUrCollision(joint_angle);
}

bool RwsInterface::setIK(Transform3D<> Ttarget)
{
    /* 
    function type: public function
    input: target position and orientation specified by an homogeneous transformation matrix.
    output: boolean representing whether the solution was found or not
    detail: move the robot to a specific configuration according to the input
    */
    Q q = this->getIK(Ttarget);
    bool ok = false;
    for(int i=0;i<6;i++)
    {
        if (q[i] > -9){
            ok = true;
            break;
        }
    }
    if(ok) this->setFK(q); 
    return ok;
    
}


Q RwsInterface::getHomeQ()
{
    /* 
    function type: public function
    input: none
    output: robot home configuration (Q)
    detail: get robot home configuration (Q)
    */
    return Q(6,1.571,-0.785,-1.571,-2.356,1.571,0.00);
    //return Q(6,1.571,0,0,0,0,0);
}


Q RwsInterface::getGoalQ()
{
    /* 
    function type: public function
    input: none
    output: goal configuration (Q)
    detail: get goal configuration (Q)
    */
    Transform3D<> target = Kinematics::frameTframe(this->urbase, this->goal, this->state);
    target = target * Transform3D<>(Vector3D<>(0,0,0),Rotation3D<>(RPY<>(0.0,0.0,3.14/2)));
    Transform3D<> ik( Vector3D<>(target.P()(0),target.P()(1),this->grasping_z), target.R());
    return this->getIK(ik);
}

Transform3D<> RwsInterface::worldTobase(Transform3D<> H)
{
    Transform3D<> Hbw = Kinematics::frameTframe(this->urbase, this->wc->getWorldFrame(), this->state);
    return Hbw*H;
}

Transform3D<> RwsInterface::baseToworld(Transform3D<> H)
{
    Transform3D<> Hbw = Kinematics::frameTframe(this->wc->getWorldFrame(), this->urbase,  this->state);
    return Hbw*H;
}

float RwsInterface::getPoseDiff(Transform3D<> T)
{
    Transform3D<> T_ = Kinematics::frameTframe(this->wc->getWorldFrame(), this->tool, this->state);
    float error = 0.0;
    for(int i=0;i<3;i++)
    {
        error += (T.P()(i)-T_.P()(i))*(T.P()(i)-T_.P()(i));
    }
    return sqrt(error);
}

/**************************************************************************
*                              Motion Control                             *
**************************************************************************/
void RwsInterface::update()
{
    /* 
    function type: public function
    input: none
    output: none
    detail: update the simulation according to the state and object termination state
    */
    if(this->objectTermination) this->object.at(this->objectId)->moveTo(this->objectTerminatePose,this->state);  
    
    this->app->getRobWorkStudio()->setState(this->state);
    TimerUtil::sleepMs (this->t_delay);
}

void RwsInterface::delay(int ms)
{
    /* 
    function type: public function
    input: delay time (in millisecond)
    output: none
    detail: delay for a certain millisecond
    */
    TimerUtil::sleepMs (ms);
}

void RwsInterface::moveToGoal()
{
    /* 
    function type: public function
    input: none
    output: none
    detail: immediately move the robot to goal configuration (Q)
    */
    this->setFK(this->getGoalQ());
}

/**************************************************************************
*                              Path Planning                              *
**************************************************************************/
bool RwsInterface::planning(Q to, int algo, float estepsize)
{
    /* 
    function type: public function
    input: target robot configuration (Q) and planning algorithm id
    output: none
    detail: perform path planning in configurtion space and update the simulation until the robot reaches the target configuration
    */

    // initializer
    QPath path;
    Q from = this->ur->getQ(this->state);
    CollisionDetector detector(this->wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,this->ur,this->state);
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(this->ur),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner;
    switch(algo) // algorithm selection
    {
        case 0: 
            planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, estepsize, RRTPlanner::RRTBalancedBidirectional);
            break;
        case 1: 
            planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, estepsize, RRTPlanner::RRTBidirectional);
            break;
        case 2: 
            planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, estepsize, RRTPlanner::RRTConnect);
            break;
        case 3: 
            planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, estepsize, RRTPlanner::RRTBasic);
            break;
        default:
            planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, estepsize, RRTPlanner::RRTBalancedBidirectional);
            break;
    }
    
    // planning
    bool complete = planner->query(from, to, path);
    if (not complete)
    {
        return false;
    }
    // looping
    Q path_i = from;
    Q path_j = from;
    Q diff = from;
    this->moving_distance = 0.0;
    for(int i=0; i < path.size(); i++)
    {
        path_j = path[i];
        this->setFK(path_j);
        this->update();
        diff = path_j-path_i;
        this->moving_distance += diff.norm2();
        path_i = path_j;
    }
    return true;
}

bool RwsInterface::linearPlanning(Q target)
{
    /* 
    function type: public function
    input: target robot configuration (Q) and planning algorithm id
    output: none
    detail: perform linear planning  and update the simulation until the robot reaches the targets
    */

    // initializer
    Q from = this->ur->getQ(this->state);
    /*Transform3D<> coord = getFk(target);
    coord = coord.z++;
    Q abovepickUp = this->getIK(coord);*/
    Q pickUp(1.635, -1.571, -2.597, -0.545, -0.064, 0.031);
    Q abovePickup(1.635, -1.571, -2.597, -0.545, -0.064, 0.031);;
    Q inter1(1.635, -1.571, -2.597, -0.545, -0.064, 0.031);
    Q inter2(-0.096, -1.571, -2.661, -0.514, -0.064, 0.031);
    Q inter3(-0.0959757, -1.94218, -2.59649, -0.559186, -0.030997, -1.18614);
    Q inter4(-0.506006, -2.02601, -2.13599, -2.05101, -0.531994, -0.0659909);
    Q abovePlace(-0.0980177, -1.81382, -2.42362, -1.76263, -0.12802, -0.286932);
    Q place(3.077, -1.571, -2.692, -2.051, -0.064, 0.031);


    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator1 = rw::trajectory::LinearInterpolator<rw::math::Q>(from, abovePickup, 5.0);
    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator2 = rw::trajectory::LinearInterpolator<rw::math::Q>(abovePickup, pickUp, 5.0);
    //grasping
    this->setGripper(true);
    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator3 = rw::trajectory::LinearInterpolator<rw::math::Q>(pickUp, inter1, 5.0);
    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator4 = rw::trajectory::LinearInterpolator<rw::math::Q>(inter1, inter2, 5.0);
    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator5 = rw::trajectory::LinearInterpolator<rw::math::Q>(inter2, inter3, 5.0);
    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator6 = rw::trajectory::LinearInterpolator<rw::math::Q>(inter3, inter4, 5.0);
    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator7 = rw::trajectory::LinearInterpolator<rw::math::Q>(inter4, abovePlace, 5.0);
    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator8 = rw::trajectory::LinearInterpolator<rw::math::Q>(abovePlace, place, 5.0);
    //ungrasping
    this->setGripper(false);
    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator9 = rw::trajectory::LinearInterpolator<rw::math::Q>(place, abovePlace, 5.0);
    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator10 = rw::trajectory::LinearInterpolator<rw::math::Q>(abovePlace, inter4, 5.0);
    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator11 = rw::trajectory::LinearInterpolator<rw::math::Q>(inter4, inter3, 5.0);
    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator12 = rw::trajectory::LinearInterpolator<rw::math::Q>(inter3, inter2, 5.0);
    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator13 = rw::trajectory::LinearInterpolator<rw::math::Q>(inter2, inter1, 5.0);

    Q path_i = from;
    Q path_j = from;
    Q diff = from;
    this->moving_distance = 0.0;
    for(int i = 1; i <= 13; i++)
    {
        if(i == 1)
        {
            for(double i = 0; i < 5; i += 0.1)
            {
                path_j = interpolator1.x(i);
                this->setFK(path_j);
                this->update();
                diff = path_j-path_i;
                this->moving_distance += diff.norm2();
                path_i = path_j;

            }
        }
        if(i == 2)
        {
            for(double i = 0; i < 5; i += 0.1)
            {
                path_j = interpolator2.x(i);
                this->setFK(path_j);
                this->update();
                diff = path_j-path_i;
                this->moving_distance += diff.norm2();
                path_i = path_j;

            }
        }
        if(i == 3)
        {
            for(double i = 0; i < 5; i += 0.1)
            {
                path_j = interpolator3.x(i);
                this->setFK(path_j);
                this->update();
                diff = path_j-path_i;
                this->moving_distance += diff.norm2();
                path_i = path_j;

            }
        }
        if(i == 4)
        {
            for(double i = 0; i < 5; i += 0.1)
            {
                path_j = interpolator4.x(i);
                this->setFK(path_j);
                this->update();
                diff = path_j-path_i;
                this->moving_distance += diff.norm2();
                path_i = path_j;

            }
        }
        if(i == 5)
        {
            for(double i = 0; i < 5; i += 0.1)
            {
                path_j = interpolator5.x(i);
                this->setFK(path_j);
                this->update();
                diff = path_j-path_i;
                this->moving_distance += diff.norm2();
                path_i = path_j;

            }
        }
        if(i == 6)
        {
            for(double i = 0; i < 5; i += 0.1)
            {
                path_j = interpolator6.x(i);
                this->setFK(path_j);
                this->update();
                diff = path_j-path_i;
                this->moving_distance += diff.norm2();
                path_i = path_j;

            }
        }
        if(i == 7)
        {
            for(double i = 0; i < 5; i += 0.1)
            {
                path_j = interpolator7.x(i);
                this->setFK(path_j);
                this->update();
                diff = path_j-path_i;
                this->moving_distance += diff.norm2();
                path_i = path_j;

            }
        }
        if(i == 8)
        {
            for(double i = 0; i < 5; i += 0.1)
            {
                path_j = interpolator8.x(i);
                this->setFK(path_j);
                this->update();
                diff = path_j-path_i;
                this->moving_distance += diff.norm2();
                path_i = path_j;

            }
        }
        if(i == 9)
        {
            for(double i = 0; i < 5; i += 0.1)
            {
                path_j = interpolator9.x(i);
                this->setFK(path_j);
                this->update();
                diff = path_j-path_i;
                this->moving_distance += diff.norm2();
                path_i = path_j;

            }
        }
        if(i == 10)
        {
            for(double i = 0; i < 5; i += 0.1)
            {
                path_j = interpolator10.x(i);
                this->setFK(path_j);
                this->update();
                diff = path_j-path_i;
                this->moving_distance += diff.norm2();
                path_i = path_j;

            }
        }
        if(i == 11)
        {
            for(double i = 0; i < 5; i += 0.1)
            {
                path_j = interpolator11.x(i);
                this->setFK(path_j);
                this->update();
                diff = path_j-path_i;
                this->moving_distance += diff.norm2();
                path_i = path_j;

            }
        }
        if(i == 12)
        {
            for(double i = 0; i < 5; i += 0.1)
            {
                path_j = interpolator12.x(i);
                this->setFK(path_j);
                this->update();
                diff = path_j-path_i;
                this->moving_distance += diff.norm2();
                path_i = path_j;

            }
        }
        if(i == 13)
        {
            for(double i = 0; i < 5; i += 0.1)
            {
                path_j = interpolator13.x(i);
                this->setFK(path_j);
                this->update();
                diff = path_j-path_i;
                this->moving_distance += diff.norm2();
                path_i = path_j;

            }
        }
    }
    return true;
}

float RwsInterface::getMovingDistance()
{
    return this->moving_distance;
}

/**************************************************************************
*                             Gripper Control                             *
**************************************************************************/
void RwsInterface::setGripper(bool state)
{
    /* 
    function type: public function
    input: gripper state
    output: none
    detail: set gripper to open or close
    */
    Q q(1,0.055);
    if(state)
    {
        bool col = false;
        while(not col)
        {
            q[0] -= 0.001;
            this->gripper->setQ(q,this->state);
            col = this->checkGripperCollision(q);
        }
        q[0] += 0.001;
        this->gripper->setQ(q,this->state);
        Kinematics::gripFrame(this->object.at(this->objectId),this->tool,this->state);
        Transform3D<> frameBaseTGoal = Kinematics::frameTframe(this->urbase, this->tool, this->state);
        this->setGraspingHeight(frameBaseTGoal.P()(2));
    }else{
        
        this->gripper->setQ(q,this->state);
    }
}

void RwsInterface::setGraspingOrientation(int mode)
{
    /* 
    function type: public function
    input: gripper mode
    output: none
    detail: set whether the gripper has to grasp the object on the top or the front
    */
    this->grasping_orien = mode;
}

/**************************************************************************
*                               Stereo Vision                             *
**************************************************************************/

Transform3D<>  RwsInterface::sparseStereo(int method)
{
    /* 
    function type: public function
    input: method index (0 = RGB feature, 1 CARN feature)
    output: homogeneous transformation matrix of the object pose
    detail: estimate the homogeneous transformation matrix of the object pose using sparse stereo 
    */
    imwrite("../../RwsInterface/stereo/img0.png",this->getImage(0));
    imwrite("../../RwsInterface/stereo/img1.png",this->getImage(1));
    Transform3D<> Tobj = this->sparseCenter(method);
    //Tobj.P()(2) = this->sparseHeight(method);

    Transform3D<> Tbasecam = Kinematics::frameTframe(this->urbase, this->cams.at(0), this->state);
    if (this->grasping_orien == 0)
    {
        return Tbasecam*Tobj*Transform3D<>(Vector3D<>(0,0,0),Rotation3D<>(RPY<>(0.0,0.0,3.14/2)));
    }else{
        return Tbasecam*Tobj*Transform3D<>(Vector3D<>(0,0,0),Rotation3D<>(RPY<>(3.14,0,0)));
    }
    //
    
}

cv::Mat RwsInterface::defineQ(int img_width, int img_height)
{
    double cx = -img_width / 2, cy = -img_height / 2;
    double f = this->A.at(0).at<double>(0,0);
    double Tx = this->H.at(0).at<double>(0,3)-this->H.at(1).at<double>(0,3);
    cout<<"f ->>"<<to_string(f)<<endl;
    cout<<"tx ->>"<<to_string(Tx)<<endl;
	cv::Mat Q = cv::Mat::zeros(4, 4, CV_64F);
	Q.at<double>(0, 0) = 1;
	Q.at<double>(1, 1) = 1;
	Q.at<double>(0, 3) = cx;
	Q.at<double>(1, 3) = cy;
	Q.at<double>(2, 3) = f;
	Q.at<double>(3, 2) = -1 / Tx;
    return Q;
}

void RwsInterface::savePointCloud(std::string filename, cv::Mat points, cv::Mat colors, double max_z) {
    pclPoint p_default;
    pclCloud::Ptr dst(new pclCloud(points.rows, points.cols, p_default));
    for (size_t i = 0; i < points.rows; i++) {
        for (size_t j = 0; j < points.cols; j++) {
            cv::Vec3f xyz = points.at<cv::Vec3f>(i, j);
            cv::Vec3b bgr = colors.at<cv::Vec3b>(i, j);
            // Check if points are too far away, if not take them into account
            if (fabs(xyz[2]) < max_z) {
                pclPoint pn;
                pn.x = xyz[2];
                pn.y = xyz[1];
                pn.z = xyz[0];
                pn.r = bgr[2];
                pn.g = bgr[1];
                pn.b = bgr[0];
                dst->at(i, j) = pn;
            }
        }
    }
    pcl::io::savePCDFileASCII(filename, *dst);
}

Mat RwsInterface::computeDisparity(std::string option)
{
    /* 
    function type: public function
    input: two images
    output: disparity map
    detail: estimate the disparitymap using dense stereo 
    */
    cv::Mat imgL = this->getImage(1);
    cv::Mat imgR = this->getImage(0);

    Mat tmpR = imread("left_template.png");
    Mat tmpL = imread("right_template.png");

    cv::Mat nimgL = imgL;
    cv::Mat nimgR = imgR;

    for(int i = 0; i< imgL.rows; i++)
    {
        for(int j = 0; j< imgL.cols; j++)
        {
            if(imgL.at<Vec3b>(i,j)[0] == tmpL.at<Vec3b>(i,j)[0] && imgL.at<Vec3b>(i,j)[1] == tmpL.at<Vec3b>(i,j)[1] && imgL.at<Vec3b>(i,j)[2] == tmpL.at<Vec3b>(i,j)[2])
            {
                nimgL.at<Vec3b>(i,j)[0] = 0;
                nimgL.at<Vec3b>(i,j)[1] = 0;
                nimgL.at<Vec3b>(i,j)[2] = 0;
            }
            if(imgR.at<Vec3b>(i,j)[0] == tmpR.at<Vec3b>(i,j)[0] && imgR.at<Vec3b>(i,j)[1] == tmpR.at<Vec3b>(i,j)[1] && imgR.at<Vec3b>(i,j)[2] == tmpR.at<Vec3b>(i,j)[2])
            {
                nimgR.at<Vec3b>(i,j)[0] = 254;
                nimgR.at<Vec3b>(i,j)[1] = 254;
                nimgR.at<Vec3b>(i,j)[2] = 254;
            }
        }
    }

    /*Mat result(Size(imgL.cols, imgL.rows), CV_8UC1);
    imgL.convertTo(result, CV_8UC1, 1);
    imgR.convertTo(result, CV_8UC1, 1);*/

    cv::cvtColor(nimgL, nimgL, cv::COLOR_BGR2GRAY);
    cv::cvtColor(nimgR, nimgR, cv::COLOR_BGR2GRAY);

    imwrite("corrL.png",nimgL);
    imwrite("corrR.png",nimgR);

    //imgL = nimgL;
    //imgR = nimgR;

    // Setting parameters for StereoSGBM/BM algorithm
    int minDisparity = 0;
    int numDisparities = 16;
    int blockSize = 6;
    int disp12MaxDiff = 5;
    int uniquenessRatio = 5;
    int speckleWindowSize = 10;
    int speckleRange = 2;

    // Creating an object of StereoSGBM and BM algorithm
    cv::Ptr<cv::StereoSGBM> stereoSGBM = cv::StereoSGBM::create(minDisparity,numDisparities,blockSize, disp12MaxDiff,uniquenessRatio,speckleWindowSize,speckleRange);
    cv::Ptr<cv::StereoBM> stereoBM = cv::StereoBM::create(numDisparities,blockSize);
    
    // Calculating disparith using the StereoSGBM algorithm
    cv::Mat disp;
    if(option == "sgbm")
    {
        stereoSGBM->compute(imgL,imgR,disp);
    }
    else
    {
        stereoBM->compute(imgL,imgR,disp);
    }

    // Normalizing the disparity map for better visualisation
    cv::normalize(disp, disp, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    return disp;
}

void RwsInterface::denseStereo()
{
    cv::Mat disp = this->computeDisparity("sgbm");

    cv::Mat imgL = this->getImage(0);
    cv::Mat imgR = this->getImage(1);
    cv::Mat depthSensor = this->getImage(2);
    /*Mat result(Size(imgL.cols, imgL.rows), CV_16UC1);
    depthSensor.convertTo(result, CV_8UC1, 1);*/

    //cv::imshow(depthSensor);

    /*Mat result(Size(imgL.cols, imgL.rows), CV_8UC1);
    imgL.convertTo(result, CV_8UC1, 1);
    imgR.convertTo(result, CV_8UC1, 1);*/
    
    auto qMat = this->defineQ(imgL.cols, imgL.rows);

    cv::Mat points = this->reproject3D(disp, qMat);

    imshow("test", depthSensor);

    cv::Mat colors = imgL;

    double z_threshold = 500;
    this->savePointCloud("cloud.pcd", points, colors, z_threshold);
}

Mat RwsInterface::reproject3D(Mat disp, Mat Q)
{
    cv::Mat points;
    cv::reprojectImageTo3D(disp, points, Q, true);
    return points;
}

void RwsInterface::setStereoNoise(float var)
{
    this->noisevar = var;
}

Transform3D<> RwsInterface::getPose(int iterations)
{
    if(iterations == 0)
    {
        rw::math::Vector3D<> pos = rw::math::Vector3D<>(0.00935333, 0.479046, 0.33872);
        //rw::math::Rotation3D<> rotm = rw::math::Rotation3D<>(1, 0, 0, 0, 1, 0, 0, 0, 1);
        Rotation3D<> rotm(-0.447,-0.89,0,-0.82,0.41,0.37,-0.33,0.169,-0.92);
        
        Transform3D<> Tobj(pos, rotm);
        //Transform3D<> Tbasecam = Kinematics::frameTframe(this->urbase, this->cams.at(2), this->state);

        return Tobj;//Tbasecam*Tobj*Transform3D<>(Vector3D<>(0,0,0),Rotation3D<>(RPY<>(0.0,0.0,3.14/2)));
    }

    // Load
    PointCloud<PointT>::Ptr object(new PointCloud<PointT>);
    PointCloud<PointT>::Ptr scene(new PointCloud<PointT>);
    loadPCDFile("bottle.pcd", *object);
    loadPCDFile("scanner25D.pcd", *scene);

    // Compute surface normals
    NormalEstimation<PointT,PointT> ne;
    ne.setKSearch(10);
    ne.setInputCloud(object);
    ne.compute(*object);
    ne.setInputCloud(scene);
    ne.compute(*scene);


    // Compute shape features
    PointCloud<FeatureT>::Ptr object_features(new PointCloud<FeatureT>);
    PointCloud<FeatureT>::Ptr scene_features(new PointCloud<FeatureT>);
    
    SpinImageEstimation<PointT,PointT,FeatureT> spin;
    spin.setRadiusSearch(0.05);
    
    spin.setInputCloud(object);
    spin.setInputNormals(object);
    spin.compute(*object_features);
    
    spin.setInputCloud(scene);
    spin.setInputNormals(scene);
    spin.compute(*scene_features);


    // Find feature matches
    Correspondences corr(object_features->size());
    for(size_t i = 0; i < object_features->size(); ++i) 
    {
        corr[i].index_query = i;
        this->nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
    }


    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree;
    tree.setInputCloud(scene);


    // Set RANSAC parameters
    const size_t iter = iterations;
    const float thressq = 0.01 * 0.01;

    // Start RANSAC
    Matrix4f pose = Matrix4f::Identity();
    PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>);
    float penalty = FLT_MAX;
    {
        cout << "Starting RANSAC..." << endl;
        UniformGenerator<int> gen(0, corr.size() - 1);
        for(size_t i = 0; i < iter; ++i) {
            if((i + 1) % 100 == 0)
                cout << "\t" << i+1 << endl;
            // Sample 3 random correspondences
            vector<int> idxobj(3);
            vector<int> idxscn(3);
            for(int j = 0; j < 3; ++j) {
                const int idx = gen.run();
                idxobj[j] = corr[idx].index_query;
                idxscn[j] = corr[idx].index_match;
            }
            
            // Estimate transformation
            Matrix4f T;
            TransformationEstimationSVD<PointNormal,PointNormal> est;
            est.estimateRigidTransformation(*object, idxobj, *scene, idxscn, T);
            
            // Apply pose
            transformPointCloud(*object, *object_aligned, T);
            
            // Validate
            vector<vector<int> > idx;
            vector<vector<float> > distsq;
            tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
            
            // Compute inliers and RMSE
            size_t inliers = 0;
            float rmse = 0;
            for(size_t j = 0; j < distsq.size(); ++j)
                if(distsq[j][0] <= thressq)
                    ++inliers, rmse += distsq[j][0];
            rmse = sqrtf(rmse / inliers);
            
            // Evaluate a penalty function
            const float outlier_rate = 1.0f - float(inliers) / object->size();
            //const float penaltyi = rmse;
            const float penaltyi = outlier_rate;
            
            // Update result
            if(penaltyi < penalty) {
                cout << "\t--> Got a new model with " << inliers << " inliers!" << endl;
                penalty = penaltyi;
                pose = T;
            }
        }
        
        transformPointCloud(*object, *object_aligned, pose);
        

        // Compute inliers and RMSE
        vector<vector<int> > idx;
        vector<vector<float> > distsq;
        tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
        size_t inliers = 0;
        float rmse = 0;
        for(size_t i = 0; i < distsq.size(); ++i)
            if(distsq[i][0] <= thressq)
                ++inliers, rmse += distsq[i][0];
        rmse = sqrtf(rmse / inliers);
    
        /* Show result
        PCLVisualizer v("After global alignment");
        v.addPointCloud<PointT>(object_aligned, PointCloudColorHandlerCustom<PointT>(object_aligned, 0, 255, 0), "object_aligned");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.spin();
        // Print pose*/

        rw::math::Vector3D<> pos = rw::math::Vector3D<>(pose(0,3), pose(1,3), pose(2,3));
        rw::math::Rotation3D<> rotm = rw::math::Rotation3D<>(pose(0,0), pose(0,1), pose(0,2),pose(1,0), pose(1,1), pose(1,2),pose(2,0), pose(2,1), pose(2,2));
        
        Transform3D<> Tobj(pos, rotm);

        //return Tobj;

        Transform3D<> Tbasecam = Kinematics::frameTframe(this->urbase, this->cams.at(2), this->state);

        return Tbasecam*Tobj*Transform3D<>(Vector3D<>(0,0,0),Rotation3D<>(RPY<>(0.0,0.0,3.14/2)));
    }
}

Matrix4f RwsInterface::getPoseLoc()
{
    // Load
    PointCloud<PointT>::Ptr object(new PointCloud<PointT>);
    PointCloud<PointT>::Ptr scene(new PointCloud<PointT>);
    loadPCDFile("bottle.pcd", *object);
    loadPCDFile("scanner25D.pcd", *scene);
    
    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree;
    tree.setInputCloud(scene);
    
    // Set ICP parameters
    const size_t iter =  500;
    const float thressq = 0.01 * 0.01;
    
    // Start ICP
    Matrix4f pose = Matrix4f::Identity();
    PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>(*object));
    cout << "Starting ICP..." << endl;
    for(size_t i = 0; i < iter; ++i) 
    {
        // 1) Find closest points
        vector<vector<int> > idx;
        vector<vector<float> > distsq;
        tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
        
        // Threshold and create indices for object/scene and compute RMSE
        vector<int> idxobj;
        vector<int> idxscn;
        for(size_t j = 0; j < idx.size(); ++j) 
        {
            if(distsq[j][0] <= thressq) 
            {
                idxobj.push_back(j);
                idxscn.push_back(idx[j][0]);
            }
        }
        
        // 2) Estimate transformation
        Matrix4f T;
        TransformationEstimationSVD<PointNormal,PointNormal> est;
        est.estimateRigidTransformation(*object_aligned, idxobj, *scene, idxscn, T);
        
        // 3) Apply pose
        transformPointCloud(*object_aligned, *object_aligned, T);
        
        // 4) Update result
        pose = T * pose;
        
        // Compute inliers and RMSE
        tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
        size_t inliers = 0;
        float rmse = 0;
        for(size_t i = 0; i < distsq.size(); ++i)
            if(distsq[i][0] <= thressq)
                ++inliers, rmse += distsq[i][0];
        rmse = sqrtf(rmse / inliers);
    } // End timing
    
    // Show result
    PCLVisualizer v("After local alignment");
    v.addPointCloud<PointNormal>(object_aligned, PointCloudColorHandlerCustom<PointNormal>(object_aligned, 0, 255, 0), "object_aligned");
    v.addPointCloud<PointNormal>(scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0),"scene");
    v.spin();

    // Print pose
    return pose;
}

/**************************************************************************
*                             Private Methods                             *
**************************************************************************/

void RwsInterface::nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq) {
    idx = 0;
    distsq = this->dist_sq(query, target[0]);
    for(size_t i = 1; i < target.size(); ++i) {
        const float disti = this->dist_sq(query, target[i]);
        if(disti < distsq) {
            idx = i;
            distsq = disti;
        }
    }
}

inline float RwsInterface::dist_sq(const FeatureT& query, const FeatureT& target) {
    float result = 0.0;
    for(int i = 0; i < FeatureT::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }
    
    return result;
}

rw::math::Transform3D<> RwsInterface::matrix2Transform(const Eigen::Matrix4f matrix) 
{
    rw::math::Vector3D<> pos = rw::math::Vector3D<>(matrix(0,3), matrix(1,3), matrix(2,3));
    rw::math::Rotation3D<> rotm = rw::math::Rotation3D<>(matrix(0,0), matrix(0,1), matrix(0,2),
                    matrix(1,0), matrix(1,1), matrix(1,2),
                    matrix(2,0), matrix(2,1), matrix(2,2));
    
    RPY<> rpy = RPY<>(rotm);
    return rw::math::Transform3D<>(pos, rpy);
}

bool RwsInterface::checkUrCollision(Q q)
{
    /* 
    function type: private function
    input: robot configuration (Q)
    output: boolean (0 = no collision, 1 = there is a collision)
    detail: check whether our robot has any collision or not.
    */
    this->ur->setQ(q,this->state);
    CollisionDetector::QueryResult data;
    bool col = this->detector->inCollision(this->state,&data);
    if (col) // check if there are any collision or not (include the tool and target object)
    {
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) 
        {
            if(not this->checkExceptionCollision(string((*it).first->getName()),string((*it).second->getName()))) return true;  
        }
    }
    return false; // means there is no collision
}



bool RwsInterface::checkGripperCollision(Q q)
{
    /* 
    function type: private function
    input: gripper configuration (Q)
    output: boolean (0 = no collision, 1 = there is a collision)
    detail: check whether our gripper has any collision or not.
    */
    this->gripper->setQ(q,this->state);
    CollisionDetector::QueryResult data;
    bool col = this->detector->inCollision(this->state,&data);
    if (col) // check if there are any collision or not (include the tool and target object)
    {
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) 
        {
            return true;
        }
    }
    return false; // means there is no collision
}


bool RwsInterface::checkFKIK(Transform3D<> FKresult, Q IKresult,float threshold)
{
    /* 
    function type: private function
    input: the result of forward and inverse kinematics, and a small threshold (i.e., 0.5)
    output: boolean (0 = not match, 1 = there is a collisionmatch)
    detail: check whether the result from forward kinematics matches the result from inverse kinematics or not.
    */
    Transform3D<> T = getFK(IKresult);
    if (((T.P()-FKresult.P()).norm2()) > threshold)
    {
        return false; // not match
    }
    return true; // match
}

bool RwsInterface::checkExceptionCollision(string s1,string s2)
{
    /* 
    function type: private function
    input: two names of two collided frames
    output: boolean (0 = not the exception collision, 1 = the exception collision)
    detail: check whether two frame pair is excepted from the collision or not
    */
    vector<string>listofnames;
    listofnames.push_back(string("WSG50.Finger1"));
    listofnames.push_back(string("WSG50.Finger2"));
    listofnames.push_back(string("WSG50.FingerTipMount1"));
    listofnames.push_back(string("WSG50.FingerTipMount2"));


    for(int i=0;i<listofnames.size();i++)
    {
        if( (s1.compare(listofnames[i]) == 0) and (s2.compare( this->objectName.at(this->objectId)) == 0)) return true;
        if((s2.compare(listofnames[i]) == 0) and (s1.compare( this->objectName.at(this->objectId)) == 0)) return true;
    }
    return false;
}

void RwsInterface::setGraspingHeight(float z)
{
    /* 
    function type: private function
    input: grasping height
    output: none
    detail: set the grasping height
    */
    this->grasping_z = z;
}


Mat RwsInterface::rwH2cvH(Transform3D<> H)
{
    /* 
    function type: private function
    input: homogeneous transformation matrix (in robwork)
    output: homogeneous transformation matrix (in opencv matrix)
    detail: convert homogeneous transformation matrix from robwork to opencv
    */
    Mat H_ = Mat::zeros(4,4,CV_64F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<4;j++)
        {
            H_.at<double>(i,j) = H(i,j);
        }
    }
    H_.at<double>(3,3) = 1.0;
    return H_;
}

Transform3D<> RwsInterface::sparseCenter(int method)
{
    /* 
    function type: private function
    input: feature extraction method
    output: estimated homogeneous transformation matrix of the object
    detail: estimate homogeneous transformation matrix of the object using single point
    */
    string cmd("python3 ../../RwsInterface/stereo/sparseStereo.py ");
    cmd.append(to_string(method));
    cmd.append(" ");
    cmd.append(to_string(this->A.at(0).at<double>(0,0)));
    cmd.append(" ");
    cmd.append(to_string(this->H.at(0).at<double>(0,3)-this->H.at(1).at<double>(0,3)));
    cmd.append(" 1.0 100 0 1.1 ");
    cmd.append(to_string(this->noisevar));
    system(cmd.c_str());

    PointCloud<point>::Ptr cloud (new PointCloud<point>);
    vector<vector<float>> matches = this->readCSV(String("../../RwsInterface/stereo/python_output.csv"));
    cloud->points.resize(matches.size());
    float angle = 0.0;
    for (int i=0;i<matches.size();i++)
    {
        cloud->points[i].x = matches.at(i).at(0);
        cloud->points[i].y = matches.at(i).at(1);
        cloud->points[i].z = matches.at(i).at(2);
        angle = matches.at(i).at(3)*M_PI/180.0;
    }

    Transform3D<> Tobj(Vector3D<>(-cloud->points[0].x,cloud->points[0].y,-cloud->points[0].z-GRASPINGOFFSET),Rotation3D<>(0,0,-1,-1,0,0,0,1,0));
    Transform3D<> Torien(Vector3D<>(0,0,0),Rotation3D<>(RPY<>(0,-angle,0)));
    Tobj = Tobj * Torien;
    return Tobj;
}


float RwsInterface::sparseHeight(int method)
{
    /* 
    function type: private function
    input: feature extraction method
    output: estimated height of the object
    detail: estimate height of the object using multiple points
    */
    string cmd("python3 ../../RwsInterface/stereo/sparseStereo.py ");
    cmd.append(to_string(method));
    cmd.append(" ");
    cmd.append(to_string(this->A.at(0).at<double>(0,0)));
    cmd.append(" ");
    cmd.append(to_string(this->H.at(0).at<double>(0,3)-this->H.at(1).at<double>(0,3)));
    cmd.append(" 1.0 0.2 0.5 1.0 0.0");
    system(cmd.c_str());

    PointCloud<point>::Ptr cloud (new PointCloud<point>);
    vector<vector<float>> matches = this->readCSV(String("../../RwsInterface/stereo/python_output.csv"));
    cloud->points.resize(matches.size());
    float angle = 0.0;
    for (int i=0;i<matches.size();i++)
    {
        cloud->points[i].x = matches.at(i).at(0);
        cloud->points[i].y = matches.at(i).at(1);
        cloud->points[i].z = matches.at(i).at(2);
        angle = matches.at(i).at(3)*M_PI/180.0;
    }

    Transform3D<> Tobj(Vector3D<>(-cloud->points[0].x,cloud->points[0].y,-cloud->points[0].z-GRASPINGOFFSET),Rotation3D<>(0,0,-1,-1,0,0,0,1,0));
    Transform3D<> Torien(Vector3D<>(0,0,0),Rotation3D<>(RPY<>(0,-angle,0)));
    Tobj = Tobj * Torien;
    return Tobj.P()(2);
}


vector<vector<float>> RwsInterface::readCSV(String filename)
{
    /* 
    function type: private function
    input: filename
    output: data
    detail: read data from a specific csv file
    */
    // initialize
    fstream fin;
    fin.open(filename, ios::in);
    // define variable
    vector<vector<float>> ret;
    vector<float> row;
    string line, word;

    while (1) {
        row.clear();
        getline(fin, line);
        stringstream s(line);
        while (getline(s, word, ',')) {
            if (word.compare(String("x")) == 0) return ret; // end if "x" is found
            row.push_back(stof(word));
        }
        ret.push_back(row);
    }
}

visualization::PCLVisualizer::Ptr RwsInterface::simpleVis (boost::shared_ptr<visualization::PCLVisualizer> viewer,PointCloud<point>::ConstPtr cloud, int r, int g, int b,std::string const& name)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  viewer->setBackgroundColor (0, 0, 0);
  visualization::PointCloudColorHandlerCustom<point> single_color(cloud, r, g, b);
  viewer->addPointCloud<point> (cloud,single_color, name);
  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
  viewer->addCoordinateSystem (0.0);
  viewer->initCameraParameters ();
  return (viewer);
}