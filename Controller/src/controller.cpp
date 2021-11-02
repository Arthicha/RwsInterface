#include "RwsInterface.h"

#define WC_FILE "../../Project_WorkCell/Scene.wc.xml"

int main (int argc, char** argv)
{
    RwsInterface sim(WC_FILE);

    RobWorkStudioApp app ("");
    
    RWS_START (app)
    {
        sim.setup(&app);
        Mat img1 = sim.getImage(0);
        imshow("img",img1);
        //Mat img2 = sim.getImage(1);
        //imshow("left",img2);
        waitKey(0);
        app.close ();
    }RWS_END()

    return 0;
}
