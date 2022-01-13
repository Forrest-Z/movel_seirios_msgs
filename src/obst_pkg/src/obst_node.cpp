#include "obst_pkg/obst.hpp"
#include <movel_hasp_vendor/license.h>

int main(int argc, char * argv[])
{
    // change ml ()
    #ifdef MOVEL_LICENSE                                                                                                    
        MovelLicense ml(38);                                                                                                   
        if (!ml.login())                                                                                                      
        return 1;                                                                                                           
    #endif

    ros::init(argc, argv, "obst_node");

    ros::NodeHandle n;

    ObstClass ObstClassObject(&n);

    ros::spin();

    #ifdef MOVEL_LICENSE                                                                                                    
        ml.logout();          
    #endif

    return 0;
}