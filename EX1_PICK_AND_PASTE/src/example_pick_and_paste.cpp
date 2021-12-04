#include "RwsInterface.h"
#include <random>

#define WC_FILE "../../Project_WorkCell/Scene.wc.xml"
#define PLANNINGALGO 0

int randomGenerator()
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
        sim.setTargetIdx(rand);
        for (int i=0;i<3;i++) 
        {
            if(i != rand) sim.moveObject(i,i*10+10.0,0.0,0.0,0.0,0,0.0);
        }
        sim.moveUR(0.0,0.0,0.1,0.0,0.0,0.0);
        sim.update();
        // set home
        sim.setFK(sim.getHomeQ());
        sim.setGripper(false);
        sim.update();
        // estimate object pose
        Transform3D<> target = sim.sparseStereo(1);

        // move
        sim.planning(sim.getIK(target),PLANNINGALGO);
        sim.update();
        sim.setGripper(true);
        sim.update();
        sim.planning(sim.getGoalQ(),PLANNINGALGO);
        sim.setGripper(false);
        sim.update();
        sim.delay(3.0);
        sim.terminateObject();
        sim.update();
        sim.planning(sim.getHomeQ(),PLANNINGALGO);
        sim.update();
        sim.delay(2.0);
        app.close();
    }RWS_END()

    return 0;
}
