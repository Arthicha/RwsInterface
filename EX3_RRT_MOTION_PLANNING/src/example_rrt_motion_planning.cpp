#include "RwsInterface.h"
#include <random>

#define WC_FILE "../src/Project_WorkCell/Scene.wc.xml"

int randomGenerator() // random object
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

    // variables
    float ox, oy, oz = 0.0;
    float estep = 0.0;
    float dgo_ = 0.0;
    bool success = false;
    bool success_goal1 = false;
    bool success_goal2 = false;
    bool success_home = false;
    float total_distance = false;
    float total_time = false;
    float objxy[6][2] = {   {-0.4,0.3},{-0.4,0.6},
                            {0.0,0.3},{0.0,0.6},
                            {0.4,0.3},{0.4,0.6}}; // object position (worst case scenario)


    RWS_START (app)
    {
        // setup
        sim.setup(&app);
        sim.setDelay(10);
        Transform3D<> H(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(0.0,0.0,0.0)));
        for (int i=0;i<3;i++) sim.moveObject(i,i*10+10.0,0.0,0.0,0.0,0,0.0);
        sim.moveUR(0.0608913,0.104499,0.392774,0,0,0);

        for(int n_i=0;n_i<3;n_i++) // loop through n_i repretation
        {
            for(int algoid=0;algoid<4;algoid++) // loop through each algorithm
            {
                for(int estepid=0;estepid<4;estepid++) // loop through each step size
                {
                    estep = 1.0-0.2-estepid*0.2; // calculate step size = f(xxxxx)

                    for(int oxy=0;oxy<6;oxy+=1) // loop through object xy position
                    {
                        for(int ozz=0;ozz<4;ozz+= 1) // loop through object z position
                        {
                            ox = objxy[oxy][0]; // calculate/map object position
                            oy = objxy[oxy][1]; 
                            oz = 0.05 + 0.1*float(ozz);

                            for(int go=0;go<2;go++) // loop through each grasping strategy (top or side)
                            {
                                for(int dgo = 0; dgo < 6; dgo++) // loop through different object orientation
                                {
                                    // trail setup
                                    total_distance = 0.0;
                                    total_time = 0.0;
                                    const clock_t begin_time = clock();

                                    // state 1: set home
                                    sim.setFK(sim.getHomeQ());
                                    sim.setGripper(false);
                                    sim.update();

                                    dgo_ = dgo*(M_PI/6)-(M_PI/2); // calculate object oreintation = f(xxxxxx)

                                    // state 2: pick area
                                    if(go == 0)
                                    {
                                        H = Transform3D<>(Vector3D<>(ox,oy,oz),Rotation3D<>(RPY<>(0.0,0.0,M_PI)));
                                        H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(dgo_,0.0,0.0))); // top
                                    }else{
                                        H = Transform3D<>(Vector3D<>(ox,oy,oz),Rotation3D<>(RPY<>(0.0,0.0,-M_PI/2)));
                                        H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(0.0,dgo_,0.0))); // front
                                    } 
                                    success = sim.planning(sim.getIK(sim.worldTobase(H)),algoid,estep);
                                    total_distance += sim.getMovingDistance();
                                    if (success) 
                                    {
                                        sim.update();
                                    }

                                    // state 3: place area 1
                                    if(go == 0)
                                    {
                                        H = Transform3D<>(Vector3D<>(0.3,-0.5,0.05),Rotation3D<>(RPY<>(0.0,0.0,M_PI)));
                                        H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(dgo_,0.0,0.0))); // top
                                    }else{
                                        H = Transform3D<>(Vector3D<>(0.3,-0.5,0.05),Rotation3D<>(RPY<>(0.0,0.0,-M_PI/2)));
                                        H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(0.0,dgo_,0.0))); // front
                                        H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(0.0,3.14,0.0))); // flip
                                    } 
                                    success_goal1 = sim.planning(sim.getIK(sim.worldTobase(H)),algoid,estep);
                                    total_distance += sim.getMovingDistance();
                                    if (success_goal1) 
                                    {
                                        sim.update();
                                    }

                                    // state 4: place area 2
                                    if(go == 0)
                                    {
                                        H = Transform3D<>(Vector3D<>(0.3,-0.5,0.20),Rotation3D<>(RPY<>(0.0,0.0,M_PI)));
                                        H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(dgo_,0.0,0.0))); // top
                                    }else{
                                        H = Transform3D<>(Vector3D<>(0.3,-0.5,0.20),Rotation3D<>(RPY<>(0.0,0.0,-M_PI/2)));
                                        H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(0.0,dgo_,0.0))); // front
                                        H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(0.0,3.14,0.0))); // flip
                                    } 
                                    success_goal2 = sim.planning(sim.getIK(sim.worldTobase(H)),algoid,estep);
                                    total_distance += sim.getMovingDistance();
                                    if (success_goal2) 
                                    {
                                        sim.update();
                                    }

                                    // state 5: home position
                                    success_home = sim.planning(sim.getHomeQ(),algoid,estep);
                                    total_distance += sim.getMovingDistance();
                                    if (success_home) 
                                    {
                                        sim.update();
                                    }

                                    // write to csv
                                    total_time = float( clock () - begin_time )/  CLOCKS_PER_SEC;
                                    fstream file;
                                    file.open("rrt.csv", ios::app);
                                    file << "\"" << algoid << "\","; 
                                    file << "\"" << estep << "\","; 
                                    file << "\"" << ox << "\","; 
                                    file << "\"" << oy << "\","; 
                                    file << "\"" << oz << "\","; 
                                    file << "\"" << go + 0.01*dgo << "\","; 
                                    file << "\"" << success << "\","; 
                                    file << "\"" << success_goal1 << "\","; 
                                    file << "\"" << success_goal2 << "\","; 
                                    file << "\"" << success_home << "\","; 
                                    file << "\"" << total_time << "\","; 
                                    file << "\"" << total_distance << "\","; 
                                    file << endl;
                                }
                            }
                        }
                    }
                }
            }
        }
        
        app.close();
    }RWS_END()

    return 0;
}
