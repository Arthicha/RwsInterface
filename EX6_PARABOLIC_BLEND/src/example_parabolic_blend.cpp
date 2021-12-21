#include "RwsInterface.h"
#include <random>

#define WC_FILE "../src/Project_WorkCell/Scene.wc.xml"
#define PLANNINGALGO 2
#define ESTEPSIZE 0.3

int main (int argc, char** argv)
{
    RwsInterface sim(WC_FILE);
    RobWorkStudioApp app ("");
    RWS_START (app)
    {
        // setup
        sim.setup(&app);
        sim.setDelay(10);
        sim.setGraspingOrientation(0);
        //for (int i=0;i<3;i++) if(i != rand) sim.moveObject(i,i*10+10.0,0.0,0.0,0.0,0,0.0);
        //sim.moveUR(0.0608913,0.104499,0.392774,0,0,0);
        sim.update();

        // set home
        sim.setFK(sim.getHomeQ());
        sim.setGripper(false);
        sim.update();
        
        // move the object to the goal
        Q target(1.86609, -1.38378, -2.06223, -0.887552, 1.42239, 0.355471);
        sim.parabolicPlanning(target);
        sim.update();

        sim.delay(2.0);
        bool a;
        while(true)
        {
            a = false;
        }
        app.close();
    }RWS_END()

    return 0;
}
