#include "RwsInterface.h"
#include <random>
#include <fstream>

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

        std::ofstream myFile;
        myFile.open("vision.data");

        sim.setup(&app);
        sim.setDelay(10);
        sim.setGraspingOrientation(0);
        sim.moveObject(0, 0.25, 0.473, 0.11,-3.14/2, 0, 3.14/2);
        sim.update();
        for(double i=0; i<=10; i++)
        {
            cout<<i<<endl;
            sim.update();
            sim.getImage(0);
            sim.getImage(1);
            sim.getImage(2);

            sim.moveObject(0, (0.25-(i/20)), 0.473, 0.11,-3.14/2, 0, 3.14/2);

            Transform3D<> result = sim.getPose(300);
            Transform3D<> real = sim.getObjectPose(0);

            myFile<<result<<" --- "<<real<<endl;


            //Q q = sim.getIK(result);
            //cout << q << endl;
            //sim.setFK(q);
            sim.update();
        }

        myFile.close();
        app.close();
    }RWS_END()

    return 0;
}
