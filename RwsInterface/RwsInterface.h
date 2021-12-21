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
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/opengl/RenderImage.hpp>

// RobWorkStudio libraries
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

// opencv libraries
#include <opencv2/opencv.hpp>

// multi-threading libraries
#include <boost/thread.hpp>
#include <thread>   

// to run python stuff
#include <iostream>

// system
#include <math.h>
#include <cstdio>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>


// point cloud
#undef foreach
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>

/******************************************************************************
*                                     Prefix                                  *
******************************************************************************/

// standard 
using namespace std;

// RobWork
using namespace rw::core;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::trajectory;
using namespace rw::graphics;
using namespace rw::loaders;
using namespace rwlibs::simulation;
using namespace rw::invkin;
using namespace rw::proximity;
using namespace rw::pathplanning;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;
using namespace rwlibs::opengl;
using rw::sensor::Image;
using rw::graphics::SceneViewer;

// RobWorkStudio
using namespace rws;

// OpenCV
using namespace cv;

// point cloud
using namespace pcl;
using namespace pcl::common;
using namespace pcl::io;
using namespace pcl::registration;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;
using pclPoint = pcl::PointXYZRGB;
using pclCloud = pcl::PointCloud<pclPoint>;

typedef PointXYZ point;
typedef PointNormal PointT;
typedef Histogram<153> FeatureT;

/******************************************************************************
*                                     Define                                  *
******************************************************************************/

#define MAXTIME 60.
#define GRASPINGOFFSET 0.05

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
    *                            Object Manipulation                          *
    **************************************************************************/
    void moveObject(int idx, float x, float y, float z,float roll, float pitch, float yaw);
    void moveUR(float x, float y, float z,float roll, float pitch, float yaw);
    void terminateObject();
    /**************************************************************************
    *                            Kinematics Operation                         *
    **************************************************************************/
    Transform3D<> getObjectPose(int idx);
    Transform3D<> getFK(Q q);
    Transform3D<> getTCP();
    Q getIK(Transform3D<> frameBaseTGoal);
    bool setFK(Q joint_angle);
    bool setIK(Transform3D<> Ttarget);
    Q getHomeQ();
    Q getGoalQ();
    Transform3D<> worldTobase(Transform3D<> H);
    Transform3D<> baseToworld(Transform3D<> H);
    float getPoseDiff(Transform3D<> T);

    /**************************************************************************
    *                              Motion Control                             *
    **************************************************************************/
    void update();
    void delay(int ms);
    void moveToGoal();
    
    /**************************************************************************
    *                              Path Planning                              *
    **************************************************************************/
    bool planning(Q to, int algo, float estepsize);
    bool linearPlanning(Q to);
    bool parabolicPlanning(Q target);
    float getMovingDistance();

    /**************************************************************************
    *                             Gripper Control                             *
    **************************************************************************/
    void setGripper(bool state);
    void setGraspingOrientation(int mode);

    /**************************************************************************
    *                               Stereo Vision                             *
    **************************************************************************/
    Transform3D<>  sparseStereo(int method);
    void denseStereo();
    Mat computeDisparity(std::string option);
    Mat reproject3D(Mat disp, Mat Q);
    Mat defineQ(int img_width, int img_height);
    void setStereoNoise(float var);
    void savePointCloud(std::string filename, cv::Mat points, cv::Mat colors, double max_z);
    Transform3D<> getPose(int iterations);
    Matrix4f getPoseLoc();
    Transform3D<> matrix2Transform(const Matrix4f matrix);

private:

    /**************************************************************************
    *                            Private variables                            *
    **************************************************************************/

    // -----               application, workcell, and device              -----
    RobWorkStudioApp* app; // RobWorkStudio Application
    RobWorkStudio* rwstudio; // RobWorkStudio Object
    string wc_file; // workcell name
    WorkCell::Ptr wc; // workcell
    SerialDevice::Ptr ur; // UR5 robot
    TreeDevice::Ptr gripper; // wsg50 gripper
    State state; // workcell state variable

    // -----                     reference frames                        -----
    Frame* tcp; // tcp frame
    MovableFrame* urbase; // ur base frame
    Frame* tool; // tool frame
    MovableFrame* urref; // ur reference frame
    MovableFrame* goal; // goal frame
    vector<MovableFrame*> object; // vector of object frames
    vector<String> objectName; // vector of object names
    vector<Frame*> cams; // camera pointer

    // -----                     camera parameters                        -----
    double cams_f[3] = {0.0}; // camera focal length
    int cams_w[3] = {0}; // image width
    int cams_h[3] = {0}; // image height
    vector<Mat> K; // camera parameter (K)
    vector<Mat> A; // camera parameter (A)
    vector<Mat> H; // camera parameter (A)

    // -----                     control parameters                        -----
    int t_delay = 0.0; // delay time
    int grasping_orien = 0; // grasping orientation (0 = from the top, 1 = from the front)
    int objectId = 0.0; // object id (not to check collision with this)
    float grasping_z = 0.0; // grasping height
    float moving_distance = 0.0; // total moving distance
    float noisevar = 0.0; // image Gaussian variance
    bool objectTermination = false; // remove the object from the workspace
    Transform3D<> objectTerminatePose; // termination pose
    
    
    // -----              ik solver and collision detector              -----
    ClosedFormIKSolverUR::Ptr closedFormSovler;
    CollisionDetector::Ptr detector;
    

    /**************************************************************************
    *                             Private Methods                             *
    **************************************************************************/

    void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq);
    inline float dist_sq(const FeatureT& query, const FeatureT& target);

    // checking
    bool checkUrCollision(Q q);
    bool checkGripperCollision(Q q);
    bool checkExceptionCollision(string s1,string s2);
    bool checkFKIK(Transform3D<> FKresult, Q IKresult,float threshold);

    // setting
    void setGraspingHeight(float z);

    // transformation
    Mat rwH2cvH(Transform3D<> H);

    // sparse stereo
    Transform3D<> sparseCenter(int method);
    float sparseHeight(int method);

    // cpp-python communication
    vector<vector<float>> readCSV(String filename);

    // point cloud visualization
    visualization::PCLVisualizer::Ptr simpleVis (boost::shared_ptr<visualization::PCLVisualizer> viewer,PointCloud<point>::ConstPtr cloud, int r, int g, int b,std::string const& name);


    



};

#endif
