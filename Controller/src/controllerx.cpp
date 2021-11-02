#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <opencv2/opencv.hpp>
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/RobWorkStudio.hpp>
#include "SamplePlugin.hpp"

using namespace std;
using namespace cv;
using namespace rws;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::graphics;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::simulation;
using namespace rwlibs::proximitystrategies;
using rws::RobWorkStudioPlugin;

#define MAXTIME 60.
#define ESTEPSIZE 0.01

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		//cerr << "Configuration in collision: " << q << endl;
		//cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			//cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
    return true;
}


void getImage(WorkCell::Ptr _wc, rwlibs::simulation::GLFrameGrabber* _framegrabber,rw::kinematics::State _state) {
    std::vector<std::string> _cameras;
    _cameras = {"Camera_Right", "Camera_Left"};

    if (_framegrabber != NULL) {
        for( int i = 0; i < _cameras.size(); i ++)
        {
            // Get the image as a RW image
            Frame* cameraFrame = _wc->findFrame(_cameras[i]); // "Camera");
            _framegrabber->grab(cameraFrame, _state);

            const rw::sensor::Image* rw_image = &(_framegrabber->getImage());

            // Convert to OpenCV matrix.
            Mat image = Mat(rw_image->getHeight(), rw_image->getWidth(), CV_8UC3, (rw::sensor::Image*)rw_image->getImageData());

            // Convert to OpenCV image
            Mat imflip, imflip_mat;
            flip(image, imflip, 1);
            cvtColor( imflip, imflip_mat, COLOR_RGB2BGR );

            imwrite(_cameras[i] + ".png", imflip_mat );

        }
    }

}


int main(int argc, char** argv) {

    const string wcFile = "../../Project_WorkCell/Scene.wc.xml";
    const string deviceName = "UR-6-85-5-A";
    cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

    
    rw::math::Math::seed();     //Sets the random seed

    WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
    rwlibs::simulation::GLFrameGrabber* framegrabber;
    framegrabber = NULL;
    double fovy;
    int width,height;
    Frame* cameraFrame = wc->findFrame("Camera_Right");
    std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
    std::istringstream iss (camParam, std::istringstream::in);
    iss >> fovy >> width >> height;
    // Create a frame grabber
    //framegrabber = new GLFrameGrabber(width,height,fovy);
    cout << "INN" << endl;
    //SamplePlugin plugin;
    //plugin.open(wc);
    //plugin.initialize();
    //rws::RobWorkStudioPlugin plugin("SamplePluginUI", QIcon("/home/Projects/sdu_courses/rovis/project/workspace/RoviSamplePlugin/src/pa_icon.png"));
    //SceneViewer::Ptr gldrawer = plugin.getRobWorkStudio()->getView()->getSceneViewer();
    //framegrabber->init(gldrawer);
    cout << "OUT" << endl;
    

    rw::kinematics::State state;
    state = wc->getDefaultState();

    //Find the object frame
    Frame *bottle_frame = wc->findFrame("Bottle");

    Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
        cerr << "Device: " << deviceName << " not found!" << endl;
        return 0;
    }

    //getImage(wc,framegrabber,state);



	return 0;
}
