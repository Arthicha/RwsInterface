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

    this->state = this->wc->getDefaultState();

    for(int i =0;i<2;i++) // camera parameter inilization
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
    this->urbase = this->wc->findFrame("UR-6-85-5-A.Base");
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
    const string parameters = properties.get<string>("Camera");
    istringstream iss (parameters, istringstream::in);
    double fovy;
    int width;
    int height;
    iss >> fovy >> width >> height;
    double fovy_pixel = height / 2 / tan(fovy * (2*M_PI) / 360.0 / 2.0 );
    // update K matrix
    this->K.at(camNum).at<double>(0,0) = width;
    this->K.at(camNum).at<double>(1,1) = height;
    this->K.at(camNum).at<double>(0,2) = width/2.0;
    this->K.at(camNum).at<double>(1,2) = height/2.0;
    this->K.at(camNum).at<double>(2,2) = 1.0;
    // update A matrix
    this->A.at(camNum).at<double>(0,0) = fovy_pixel;
    this->A.at(camNum).at<double>(1,1) = fovy_pixel;
    this->A.at(camNum).at<double>(2,2) = 1.0;
    // update H matrix
    this->H.at(camNum) = this->rwH2cvH(Kinematics::frameTframe(this->urbase, this->cams.at(camNum), this->state));
    const SceneViewer::Ptr gldrawer = this->rwstudio->getView ()->getSceneViewer();
    const GLFrameGrabber::Ptr framegrabber = ownedPtr (new GLFrameGrabber (width, height, fovy));
    framegrabber->init (gldrawer);
    framegrabber->grab(this->cams.at(camNum), this->state);
    Image* rw_image = &(framegrabber->getImage());
    Mat img(rw_image->getHeight(),rw_image->getWidth(), CV_8UC3);
    img.data = (uchar*)rw_image->getImageData();
    Mat result;
    cvtColor( img, result, CV_BGR2RGB );
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
    cout << qs.size() << " size" << endl;
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

void RwsInterface::setFK(Q joint_angle)
{
    /* 
    function type: public function
    input: set of joint angles in configuration space (Q)
    output: none
    detail: move the robot to a specific configuration according to the input
    */
    this->ur->setQ(joint_angle,this->state);
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
void RwsInterface::planning(Q to, int algo)
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
            planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, ESTEPSIZE, RRTPlanner::RRTBalancedBidirectional);
            break;
        case 1: 
            planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, ESTEPSIZE, RRTPlanner::RRTBidirectional);
            break;
        case 2: 
            planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, ESTEPSIZE, RRTPlanner::RRTConnect);
            break;
        case 3: 
            planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, ESTEPSIZE, RRTPlanner::RRTBasic);
            break;
        default:
            planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, ESTEPSIZE, RRTPlanner::RRTBalancedBidirectional);
            break;
    }
    
    // planning
    planner->query(from, to, path);

    // looping
    for(int i=0; i < path.size(); i++)
    {
        this->setFK(path[i]);
        this->update();
    }
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
    imwrite("../../RwsInterface/img0.png",this->getImage(0));
    imwrite("../../RwsInterface/img1.png",this->getImage(1));
    Transform3D<> Tobj = this->sparseCenter(method);
    Tobj.P()(2) = this->sparseHeight(method);

    Transform3D<> Tbasecam = Kinematics::frameTframe(this->urbase, this->cams.at(0), this->state);
    return Tbasecam*Tobj*Transform3D<>(Vector3D<>(0,0,0),Rotation3D<>(RPY<>(0.0,0.0,3.14/2)));
}

Mat RwsInterface::computeDisparity()
{
    /* 
    function type: public function
    input: two images
    output: disparity map
    detail: estimate the disparitymap using dense stereo 
    */
    Mat imgL = this->getImage(0);
    Mat imgR = this->getImage(1);
    // Setting parameters for StereoSGBM algorithm
    int minDisparity = 0;
    int numDisparities = 100;
    int blockSize = 10;
    int disp12MaxDiff = 1;
    int uniquenessRatio = 10;
    int speckleWindowSize = 10;
    int speckleRange = 8;

    // Creating an object of StereoSGBM algorithm
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(minDisparity,numDisparities,blockSize, disp12MaxDiff,uniquenessRatio,speckleWindowSize,speckleRange);

    // Calculating disparith using the StereoSGBM algorithm
    cv::Mat disp;
    sgbm->compute(imgL,imgR,disp);

    // Normalizing the disparity map for better visualisation
    cv::normalize(disp, disp, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    return disp;
}

/**************************************************************************
*                             Private Methods                             *
**************************************************************************/

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
    string cmd("python3 ../../RwsInterface/sparseStereo.py ");
    cmd.append(to_string(method));
    cmd.append(" ");
    cmd.append(to_string(this->A.at(0).at<double>(0,0)));
    cmd.append(" ");
    cmd.append(to_string(this->H.at(0).at<double>(0,3)-this->H.at(1).at<double>(0,3)));
    cmd.append(" 1.0 100 0 1.1");
    system(cmd.c_str());

    PointCloud<point>::Ptr cloud (new PointCloud<point>);
    vector<vector<float>> matches = this->readCSV(String("../../RwsInterface/python_output.csv"));
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
    string cmd("python3 ../../RwsInterface/sparseStereo.py ");
    cmd.append(to_string(method));
    cmd.append(" ");
    cmd.append(to_string(this->A.at(0).at<double>(0,0)));
    cmd.append(" ");
    cmd.append(to_string(this->H.at(0).at<double>(0,3)-this->H.at(1).at<double>(0,3)));
    cmd.append(" 1.0 0.33 0.5 1.1");
    system(cmd.c_str());

    PointCloud<point>::Ptr cloud (new PointCloud<point>);
    vector<vector<float>> matches = this->readCSV(String("../../RwsInterface/python_output.csv"));
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