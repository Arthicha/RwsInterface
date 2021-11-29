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
    this->cams.push_back(this->wc->findFrame ("Camera_Right"));
    this->cams.push_back(this->wc->findFrame ("Camera_Left"));


    for(int i=0;i<this->cams.size();i++)
    {
        // get camera parameters
    }

    this->state = this->wc->getDefaultState();
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
    this->tool = this->wc->findFrame("Tool");
    this->urbase = this->wc->findFrame("UR-6-85-5-A.Base");
    this->object.push_back(this->wc->findFrame("Bottle"));
    this->object.push_back(this->wc->findFrame("Square"));
    this->object.push_back(this->wc->findFrame("Cylinder"));
    this->objectName.push_back("Bottle");
    this->objectName.push_back("Square");
    this->objectName.push_back("Cylinder");
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

    const SceneViewer::Ptr gldrawer = this->rwstudio->getView ()->getSceneViewer();
    const GLFrameGrabber::Ptr framegrabber = ownedPtr (new GLFrameGrabber (width, height, fovy));
    framegrabber->init (gldrawer);
    framegrabber->grab(this->cams.at(camNum), this->state);
    Image* rw_image = &(framegrabber->getImage());
    rw_image->saveAsPPM ("image.ppm");
    Mat img = imread("image.ppm");
    return img;
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
    return Kinematics::frameTframe(this->urbase, this->object[idx], this->state);;
}


/**************************************************************************
*                              Motion Control                             *
**************************************************************************/

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
    Transform3D<> frameTcpTRobotTcp = Kinematics::frameTframe(this->tool, this->tcp, this->state);
    Transform3D<> targetAt = frameBaseTGoal* frameTcpTRobotTcp ;

    // solve IK numerically
    vector<Q> qs = this->closedFormSovler->solve(targetAt, this->state);

    // soluton selection
    Q q_best(6,0.0,0.0,0.0,0.0,0.0);
    float dist_best = 1e10;
    float diff = 0.0;
    for (int i=0;i<qs.size();i++)
    {
        if(not this->checkCollision(qs.at(i))) // collision test
        {
            if(this->checkFKIK(targetAt,qs.at(i),0.5)) // accuracy test
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
    detail: move the robot to a specific configuration according to the input and 
            update robwork studio
    */
    this->ur->setQ(joint_angle,this->state);
    this->app->getRobWorkStudio()->setState(this->state);
    TimerUtil::sleepMs (this->t_delay);
}



void RwsInterface::setIK(Transform3D<> Ttarget)
{
    /* 
    function type: public function
    input: target position and orientation specified by an homogeneous transformation matrix.
    output: none
    detail: move the robot to a specific configuration according to the input and 
            update robwork studio
    */
    this->setFK(this->getIK(Ttarget));
}


/**************************************************************************
*                             Private Methods                             *
**************************************************************************/

bool RwsInterface::checkCollision(Q q)
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
            if ((((*it).first->getName() != "ToolMount") or ((*it).second->getName() != this->objectName.at(this->objectId)))
                and (((*it).second->getName() != "ToolMount") or ((*it).first->getName() != this->objectName.at(this->objectId)))) 
            {
                return true; // means there is at leat one collision
            }
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