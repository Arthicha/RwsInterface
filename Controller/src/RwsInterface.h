/*
interface.h
This class is for interfacing RobWorkStudio, C++ programs, and OpenCV.
Author: Arthicha Srisuchinnawong
Email: arsri21@student.sdu.dk
*/

#ifndef RWSINTERFACE_H
#define RWSINTERFACE_H

/******************************************************************************
*                             Include Libraries                               *
******************************************************************************/

// RobWork libraries
#include <rw/rw.hpp>
#include <rw/core/PropertyMap.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

// RobWorkStudio libraries
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

// opencv libraries
#include <opencv2/opencv.hpp>

// multi-threading libraries
#include <boost/thread.hpp>
#include <thread>   

/******************************************************************************
*                                     Prefix                                  *
******************************************************************************/

// standard 
using namespace std;

// RobWork
using namespace rw::core;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::graphics;
using namespace rw::loaders;
using namespace rwlibs::simulation;
using namespace rw::invkin;
using namespace rw::proximity;
using namespace rwlibs::proximitystrategies;
using rw::sensor::Image;

// RobWorkStudio
using namespace rws;

// OpenCV
using namespace cv;

using rw::graphics::SceneViewer;

/******************************************************************************
*                                  RwsInterface                               *
******************************************************************************/

class RwsInterface
{
public:

    /**************************************************************************
    *                      Setup and Initialization                           *
    **************************************************************************/
    RwsInterface(const string &_wc_file);
    virtual ~RwsInterface();
    void setup(RobWorkStudioApp* _app);
    void setDelay(int t);
    void setTargetIdx(int idx);

    /**************************************************************************
    *                             Camera Interface                            *
    **************************************************************************/
    Mat getImage(int camNum);

    /**************************************************************************
    *                            Kinematics Operation                         *
    **************************************************************************/
    Transform3D<> getObjectPose(int idx);

    /**************************************************************************
    *                              Motion Control                             *
    **************************************************************************/
    void setFK(Q joint_angle);
    void setIK(Transform3D<> Ttarget);
    Q getIK(Transform3D<> frameBaseTGoal);
    Transform3D<> getFK(Q q);

    /**************************************************************************
    *                            Public variables                             *
    **************************************************************************/
    State state; // state variable
private:

    /**************************************************************************
    *                            Private variables                            *
    **************************************************************************/
    WorkCell::Ptr wc; // workcell
    SerialDevice::Ptr ur; // UR5 robot 
        
    double cams_f[2] = {0.0}; // camera focal length
    int cams_w[2] = {0}; // image width
    int cams_h[2] = {0}; // image height

    int t_delay = 0.0; // delay time
    int objectId = 0.0; // object id (not to check collision with this)
    string wc_file; // workcell name

    RobWorkStudioApp* app; // RobWorkStudio Application
    RobWorkStudio* rwstudio; // RobWorkStudio Object

    Frame* tcp; // tcp frame
    Frame* urbase; // ur base
    Frame* tool; // tool frame
    vector<Frame*> object; // vector of object frames
    vector<String> objectName; // vector of object names
    vector<Frame*> cams; // camera pointer

    

    ClosedFormIKSolverUR::Ptr closedFormSovler;
    CollisionDetector::Ptr detector;

    /**************************************************************************
    *                             Private Methods                             *
    **************************************************************************/

    bool checkCollision(Q q);
    bool checkFKIK(Transform3D<> FKresult, Q IKresult,float threshold);


    



};

#endif