#include "RwsInterface.h"
#include <random>

#define WC_FILE "../src/Project_WorkCell/Scene.wc.xml"

int randomGenerator() // generate random integer between 0, 1, and 2
{
    random_device r;
    default_random_engine e1(r());
    uniform_int_distribution<int> uniform_dist(0, 2);
    return uniform_dist(e1);
}

float randomFloatGenerator() // generate random float between -1 and 1
{
    random_device r;
    default_random_engine e1(r());
    uniform_real_distribution<float> uniform_dist(-1, 1);
    return uniform_dist(e1);
}

int main (int argc, char** argv)
{
    RwsInterface sim(WC_FILE);
    RobWorkStudioApp app ("");

    // define tested object location (worst case scenario)
    float objectpo[8][3] = {{0.2,0.45,0.15},{-0.2,0.45,0.15},{0.2,0.65,0.15},{-0.2,0.65,0.15},
                        {0.2,0.45,0.3},{-0.2,0.45,0.3},{0.2,0.65,0.3},{-0.2,0.65,0.3}};

    RWS_START (app)
    {
        // setup
        sim.setup(&app);
        sim.setDelay(10);
        sim.setGraspingOrientation(0);
        sim.moveUR(0.0608913,0.104499,0.392774,0,0,0);
        sim.update();
        sim.setFK(sim.getHomeQ());
        sim.setGripper(false);
        sim.update();


        for (int j=0;j<5;j++) // loop through each noise
        {
            float noisevar = float(j)*5; // noise = equation(xxxxx)
            sim.setStereoNoise(noisevar); // set noise

            for (int i=0;i<3;i++) // loop through each object
            {
                sim.setTargetIdx(i); // select object to be pick
                for (int obpi=0;obpi<8;obpi+=1) // loop through each tested location
                {
                    for (int oo2=0;oo2<2;oo2++) // loop through each grasping strategy (top and side)
                    {
                        for(int repeat=0;repeat<5;repeat++) // repeat
                        {
                            float oo = randomFloatGenerator()*3.14; // random object oreintation

                            for(int j=0;j<3;j++) sim.moveObject(j,10+j*10,0,0,0,0,0); // remove all the object

                            if (oo2==0) // use only the selected object and set the object pose
                            {
                                sim.moveObject(i,objectpo[obpi][0],objectpo[obpi][1],objectpo[obpi][2],oo,0,0);
                            }else{
                                sim.moveObject(i,objectpo[obpi][0],objectpo[obpi][1],objectpo[obpi][2],oo,0,M_PI/2);
                            }
                            
                            sim.update(); // update the simulation

                            // estimate object pose
                            Transform3D<> target = sim.sparseStereo(0);

                            // calculate positional error
                            Vector3D<> diffXY = sim.getObjectPose(i).P()-target.P(); diffXY(2) = 0.0;
                            Vector3D<> diffZ = sim.getObjectPose(i).P()-target.P(); diffZ(0) = 0.0; diffZ(1) = 0.0;

                            // calculate orientation error
                            float diffR = 0.0;
                            if (((oo2 != 0) or (i != 0)) and ((oo2 != 1) or (i!=2))) diffR = abs(oo - RPY<>(target.R())(0));

                            // write to csv
                            fstream file;
                            file.open("sparse_analysis_noise.csv", ios::app);
                            file << "\"" << noisevar << "\","; 
                            file << "\"" << i+0.1*repeat << "\","; 
                            file << "\"" << obpi << "\","; 
                            file << "\"" << oo << "\","; 
                            file << "\"" << oo2 << "\","; 
                            file << "\"" << diffXY.norm2() << "\","; 
                            file << "\"" << diffZ.norm2() << "\","; 
                            file << "\"" << diffR << "\","; 
                            file << endl;
                        }
                    }
                }
            }
        }
        app.close();
    }RWS_END()

    return 0;
}
