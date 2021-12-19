#include "RwsInterface.h"
#include <random>

#define WC_FILE "../src/Project_WorkCell/Scene.wc.xml"
#define PLANNINGALGO 2
#define ESTEPSIZE 0.3

int randomGenerator() // random selected object
{
    random_device r;
    default_random_engine e1(r());
    uniform_int_distribution<int> uniform_dist(0, 2);
    return uniform_dist(e1);
}

int main (int argc, char** argv)
{
    int rand = randomGenerator();
    RwsInterface sim(WC_FILE);
    RobWorkStudioApp app ("");
    RWS_START (app)
    {
        // setup
        sim.setup(&app);
        //sim.setDelay(10);
        /*sim.setGraspingOrientation(0);
        sim.setTargetIdx(rand);
        for (int i=0;i<3;i++) if(i != rand) sim.moveObject(i,i*10+10.0,0.0,0.0,0.0,0,0.0);
        sim.moveUR(0.0608913,0.104499,0.392774,0,0,0);
        sim.update();

        // set home
        sim.setFK(sim.getHomeQ());
        sim.setGripper(false);
        sim.update();*/

        // calculate disparity map and estimate object pose
        cv::Mat disp = sim.computeDisparity("sgbm");
        sim.denseStereo();
        Mat result(Size(disp.cols, disp.rows), CV_8UC1);
        disp.convertTo(result, CV_8UC1, 1);
        cv::imshow("Stereo", disp);
        cv::imwrite("disparity.png", disp);
        cv::imwrite("depthSensor.png", sim.getImage(2));
        cv::imwrite("left.png", sim.getImage(0));
        cv::imwrite("right.png", sim.getImage(1));
        //Transform3D<> target = sim.sparseStereo(1);
        
        /* move to the object 
        sim.planning(sim.getIK(target),PLANNINGALGO,ESTEPSIZE);
        sim.update();
        // grasp the object
        sim.setGripper(true);
        sim.update();
        // move to place area/location
        sim.planning(sim.getGoalQ(),PLANNINGALGO,ESTEPSIZE);
        sim.update();
        // place the object
        sim.setGripper(false);
        sim.terminateObject();
        sim.update();
        sim.delay(2.0);
        // move to home configuration
        sim.planning(sim.getHomeQ(),PLANNINGALGO,ESTEPSIZE);
        sim.update();
        sim.delay(2.0);
        */
        app.close();
    }RWS_END()

    return 0;
}
