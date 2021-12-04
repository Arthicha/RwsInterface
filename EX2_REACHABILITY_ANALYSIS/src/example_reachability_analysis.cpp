#include "RwsInterface.h"
#include <random>

#define WC_FILE "../src/Project_WorkCell/Scene.wc.xml"


float randomGenerator()
{
    random_device r;
    default_random_engine e1(r());
    uniform_real_distribution<float> uniform_dist(-1, 1);
    return uniform_dist(e1);
}

int main (int argc, char** argv)
{
    float randx, randy, randz = 0.0;
    float ox, oy, oz = 0.0;
    bool success = false;
    bool success_goal1 = false;
    bool success_goal2 = false;
    int urpose = 0;
    float objxy[6][2] = {   {-0.4,0.3},{-0.4,0.6},
                            {0.0,0.3},{0.0,0.6},
                            {0.4,0.3},{0.4,0.6}};
    float urorien[6][3] = {{0,0,0},{0,0,M_PI/2},{0,0,-M_PI/2},{0,0,M_PI},{0,M_PI/2,0},{0,-M_PI/2,0}};

    RwsInterface sim(WC_FILE);
    RobWorkStudioApp app ("");
    RWS_START (app)
    {
        // setup
        sim.setup(&app);
        sim.setDelay(10);
        Transform3D<> H(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(0.0,0.0,0.0)));
        for (int i=0;i<3;i++) 
        {
            sim.moveObject(i,i*10+10.0,0.0,0.0,0.0,0,0.0);
        }
        while(1) // random robot position
        {
            urpose += 1;
            randx = randomGenerator()*0.5;
            randy = randomGenerator()*0.5;
            randz = (randomGenerator()+1.0)*0.5;
            for (int r=0;r<6;r++) // for all robot orientation
            {
                if(1)
                {
                    if(1)
                    {
                        sim.moveUR(randx,randy,randz,urorien[r][0],urorien[r][1],urorien[r][2]);
                        cout << urpose << endl;
                        for(int oxy=0;oxy<6;oxy+=1)
                        {
                            for(int ozz=0;ozz<4;ozz+= 1)
                            {
                                ox = objxy[oxy][0]; //x = -+ 0.4
                                oy = objxy[oxy][1]; //y = 0.3 0.6
                                oz = 0.1 + 0.1*float(ozz); //z = 0.15-0.7
                                // goal 0.3 0.5 0.15 and 0.3

                                for(int go=0;go<2;go++)
                                {
                                    for(int dgo = 0; dgo < 6; dgo++) // for all gripper orientation (top and front)
                                    {
                                        
                                        float dgo_ = dgo*(M_PI/6)-(M_PI/2); // for all minor orientation
                                        if(go == 0)
                                        {
                                            H = Transform3D<>(Vector3D<>(ox,oy,oz),Rotation3D<>(RPY<>(0.0,0.0,M_PI)));
                                            H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(dgo_,0.0,0.0))); // top
                                        }else{
                                            H = Transform3D<>(Vector3D<>(ox,oy,oz),Rotation3D<>(RPY<>(0.0,0.0,-M_PI/2)));
                                            H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(0.0,dgo_,0.0))); // front
                                        } 
                                        success = sim.setIK(sim.worldTobase(H));
                                        sim.update();

                                        if(go == 0)
                                        {
                                            H = Transform3D<>(Vector3D<>(0.3,-0.5,0.15),Rotation3D<>(RPY<>(0.0,0.0,M_PI)));
                                            H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(dgo_,0.0,0.0))); // top
                                        }else{
                                            H = Transform3D<>(Vector3D<>(0.3,-0.5,0.15),Rotation3D<>(RPY<>(0.0,0.0,-M_PI/2)));
                                            H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(0.0,dgo_,0.0))); // front
                                            H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(0.0,3.14,0.0))); // flip
                                        } 
                                        success_goal1 = sim.setIK(sim.worldTobase(H));
                                        sim.update();

                                        if(go == 0)
                                        {
                                            H = Transform3D<>(Vector3D<>(0.3,-0.5,0.30),Rotation3D<>(RPY<>(0.0,0.0,M_PI)));
                                            H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(dgo_,0.0,0.0))); // top
                                        }else{
                                            H = Transform3D<>(Vector3D<>(0.3,-0.5,0.30),Rotation3D<>(RPY<>(0.0,0.0,-M_PI/2)));
                                            H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(0.0,dgo_,0.0))); // front
                                            H = H*Transform3D<>(Vector3D<>(0.0,0.0,0.0),Rotation3D<>(RPY<>(0.0,3.14,0.0))); // flip
                                        } 
                                        success_goal2 = sim.setIK(sim.worldTobase(H));
                                        sim.update();


                                        // write data to csv
                                        fstream file;
                                        file.open("data.csv", ios::app);
                                        file << "\"" << urpose << "\","; 
                                        file << "\"" << randx << "\","; 
                                        file << "\"" << randy << "\","; 
                                        file << "\"" << randz << "\","; 
                                        file << "\"" << urorien[r][0] << "\","; 
                                        file << "\"" << urorien[r][1] << "\","; 
                                        file << "\"" << urorien[r][2] << "\","; 
                                        file << "\"" << ox << "\","; 
                                        file << "\"" << oy << "\","; 
                                        file << "\"" << oz << "\","; 
                                        file << "\"" << go + 0.01*dgo << "\","; 
                                        file << "\"" << success << "\","; 
                                        file << "\"" << success_goal1 << "\","; 
                                        file << "\"" << success_goal2 << "\","; 
                                        file << endl;
                                    }
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
