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
    this->wc_file = _wc_file;
    this->wc = WorkCellLoader::Factory::load (_wc_file);
    this->ur = this->wc->findDevice("UR-6-85-5-A");
    this->cams.push_back(this->wc->findFrame ("Camera_Right"));
    this->cams.push_back(this->wc->findFrame ("Camera_Left"));


    for(int i=0;i<this->cams.size();i++)
    {
        
    }

    this->state = this->wc->getDefaultState();
    
}

RwsInterface::~RwsInterface()
{
    
}

void RwsInterface::setup(RobWorkStudioApp* _app)
{
    this->app = _app;
    this->rwstudio = this->app->getRobWorkStudio ();
    this->rwstudio->postOpenWorkCell(this->wc_file);
    TimerUtil::sleepMs (1000);
    //const SceneViewer::Ptr this->gldrawer = this->rwstudio->getView ()->getSceneViewer ();
    //const GLFrameGrabber::Ptr this->framegrabber = ownedPtr (new GLFrameGrabber (width, height, fovy));
}


/*****************************************************************************
*                                 Camera Interface                           *
*****************************************************************************/

Mat RwsInterface::getImage(int camNum)
{
    const PropertyMap& properties = this->cams.at(camNum)->getPropertyMap ();
    const string parameters = properties.get<string>("Camera");
    istringstream iss (parameters, istringstream::in);
    double fovy;
    int width;
    int height;
    iss >> fovy >> width >> height;
    this->rwstudio = this->app->getRobWorkStudio ();
    this->rwstudio->postOpenWorkCell(this->wc_file);
    const SceneViewer::Ptr gldrawer = this->rwstudio->getView ()->getSceneViewer();
    const GLFrameGrabber::Ptr framegrabber = ownedPtr (new GLFrameGrabber (width, height, fovy));
    framegrabber->init (gldrawer);
    framegrabber->grab(this->cams.at(camNum), this->state);
    const Image* rw_image = &(framegrabber->getImage());
    return Mat(rw_image->getHeight(), rw_image->getWidth(), CV_8UC3, (Image*)rw_image->getImageData());
}