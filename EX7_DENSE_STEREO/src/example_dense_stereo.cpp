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
        sim.setDelay(10);
        sim.setGraspingOrientation(0);
        /*sim.setTargetIdx(rand);
        for (int i=0;i<3;i++) if(i != rand) sim.moveObject(i,i*10+10.0,0.0,0.0,0.0,0,0.0);
        
        sim.update();

        // set home
        sim.setFK(sim.getHomeQ());
        sim.setGripper(false);
        sim.update();*/

        // calculate disparity map and estimate object pose
        sim.moveUR(0.0608913,0.104499,0.392774,0,0,0);
        sim.update();
        sim.getImage(0);
        sim.getImage(1);
        sim.getImage(2);

        Transform3D<> result = sim.getPose(0);
        
        cout<<result<<endl;

        Q q = sim.getIK(result);
        cout << q << endl;
        sim.setFK(q);
        sim.update();
        

        while(true)
        {
            bool a = false;
        }
        

        //Transform3D<> target = Transform3D(pose);


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
