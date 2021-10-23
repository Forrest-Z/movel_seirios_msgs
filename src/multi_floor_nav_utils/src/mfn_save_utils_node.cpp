#include "multi_floor_nav_utils/mfn_save_utils.hpp"
#include <movel_hasp_vendor/license.h>

int main(int argc, char * argv[])
{
    #ifdef MOVEL_LICENSE                                                                                                    
        MovelLicense ml(38);                                                                                                   
        if (!ml.login())                                                                                                      
        return 1;                                                                                                           
    #endif

    ros::init(argc, argv, "mfn_save_utils_node");

    ros::NodeHandle n;

    MFNSaveUtils MFNSaveUtilsObject(&n);

    ros::spin();

    #ifdef MOVEL_LICENSE                                                                                                    
        ml.logout();          
    #endif

    return 0;
}