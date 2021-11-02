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
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::graphics;
using namespace rw::loaders;
using rw::sensor::Image;
using namespace rwlibs::simulation;

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

    /**************************************************************************
    *                             Camera Interface                            *
    **************************************************************************/
    Mat getImage(int camNum);

    /**************************************************************************
    *                            Public variables                             *
    **************************************************************************/

private:

    /**************************************************************************
    *                            Private variables                            *
    **************************************************************************/
    WorkCell::Ptr wc; // workcell
    Device::Ptr ur; // UR5 robot 
    State state; // state variable

    vector<Frame*> cams; // camera pointer
    double cams_f[2] = {0.0}; // camera focal length
    int cams_w[2] = {0}; // image width
    int cams_h[2] = {0}; // image height

    string wc_file; // workcell name
    RobWorkStudioApp* app; // RobWorkStudio Application
    RobWorkStudio* rwstudio; // RobWorkStudio Object



};

#endif