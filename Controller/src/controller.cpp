#include "RwsInterface.h"

#define WC_FILE "../../Project_WorkCell/Scene.wc.xml"

int main (int argc, char** argv)
{
    RwsInterface sim(WC_FILE);

    RobWorkStudioApp app ("");
    
    RWS_START (app)
    {
        sim.setup(&app);
        sim.setDelay(200);
        sim.setTargetIdx(0);
        
        /*--------- get images ---------*/
        Mat img1 = sim.getImage(0);
        imshow("img1",img1);
        Mat img2 = sim.getImage(1);
        imshow("img2",img2);
        /*------------------------------*/

        /*----- Compute Disparity ------*/
        Mat disp = sim.computeDisparity(img1, img2);
        imshow("Disparity Map",disp);
        waitKey(0);
        /*------------------------------*/

        /*
        while(1)
        {
            sim.setIK(sim.getObjectPose(0)); 
        }
        */

        app.close ();
    }RWS_END()

    return 0;
}
