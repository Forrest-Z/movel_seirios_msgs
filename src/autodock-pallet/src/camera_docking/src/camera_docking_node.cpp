#include <ros/ros.h>
#include <camera_docking/camera_docking.h>
#include <movel_hasp_vendor/license.h>

using namespace std;
using namespace cv;


int main(int argc, char ** argv) {

    #ifdef MOVEL_LICENSE                                                                                                    
        MovelLicense ml;                                                                                                   
        if (!ml.login())                                                                                                      
            return 1;                                                                                                           
    #endif
    ros::init(argc, argv, "camera_docking");

    FiducialsNode* node = new FiducialsNode();
    ros::spin();

    #ifdef MOVEL_LICENSE                                                                                                    
        ml.logout();          
    #endif       

    return 0;
}
