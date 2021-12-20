#include "RwsInterface.h"
#include <random>

#define WC_FILE "../src/Project_WorkCell/Scene.wc.xml"
#define MX 360
#define STEP 30

float mapQ(int index)
{
    float idx = (float)index;
    float mx = (float) 360;
    return 3.1415926*2*idx/mx;
}


int main (int argc, char** argv)
{
    bool success = false;
    RwsInterface sim(WC_FILE);
    RobWorkStudioApp app ("");
    RWS_START (app)
    {
        // setup
        sim.setup(&app);
        sim.setDelay(10);
        for (int i=0;i<3;i++) sim.moveObject(i,i*10+10.0,0.0,0.0,0.0,0,0.0);
        sim.moveUR(0.0608913,0.104499,0.392774,0,0,0);
        for(int q1_i=0;q1_i<MX;q1_i+=STEP)
        {
            cout << q1_i << endl;
            for(int q2_i=0;q2_i<MX;q2_i+=STEP)
            {
                for(int q3_i=0;q3_i<MX;q3_i+=STEP)
                {
                    for(int q4_i=0;q4_i<MX;q4_i+=STEP)
                    {
                        for(int q5_i=0;q5_i<MX;q5_i+=STEP)
                        {
                            for(int q6_i=0;q6_i<MX;q6_i+=STEP)
                            {
                                Q q(6,mapQ(q1_i),mapQ(q2_i),mapQ(q3_i),mapQ(q4_i),mapQ(q5_i),mapQ(q6_i));
                                success = sim.setFK(q);
                                sim.update();
                                Transform3D<> T = sim.getTCP();
                                RPY<> R = RPY<>(T.R());
                                
                                fstream file;
                                file.open("ws.csv", ios::app);
                                file << "\"" << mapQ(q1_i) << "\","; 
                                file << "\"" << mapQ(q2_i) << "\","; 
                                file << "\"" << mapQ(q3_i) << "\","; 
                                file << "\"" << mapQ(q4_i) << "\","; 
                                file << "\"" << mapQ(q5_i) << "\","; 
                                file << "\"" << mapQ(q6_i) << "\","; 
                                file << "\"" << T.P()(0) << "\","; 
                                file << "\"" << T.P()(1) << "\","; 
                                file << "\"" << T.P()(2) << "\","; 
                                file << "\"" << R(0) << "\","; 
                                file << "\"" << R(1) << "\","; 
                                file << "\"" << R(2) << "\","; 
                                file << "\"" << success << "\","; 
                                file << endl;
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
