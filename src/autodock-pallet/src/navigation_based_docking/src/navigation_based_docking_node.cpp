
#include <ros/ros.h>
#include "navigation_based_docking/navigation_based_docking.hpp"
#include "navigation_based_docking/StartAutoDock.h"
#include <movel_hasp_vendor/license.h>

using namespace std;


int main(int argc, char ** argv) {

    #ifdef MOVEL_LICENSE                                                                                                    
        MovelLicense ml;                                                                                                   
        if (!ml.login())                                                                                                      
            return 1;                                                                                                           
    #endif
    ros::init(argc, argv, "navigation_based_docking");
    ros::NodeHandle nodeHandle("~");

    Docking docking(nodeHandle);
    docking.startAutoDock = nodeHandle.advertiseService("NavigationDocking", &Docking::autoDock, &docking);
    docking.pause_sub_ = nodeHandle.subscribe("/autodock_pallet/pauseDocking", 1, &Docking::pauseCb, &docking);
    docking.resume_sub_ = nodeHandle.subscribe("/autodock_pallet/resumeDocking", 1, &Docking::resumeCb, &docking);
    docking.cancel_sub_ = nodeHandle.subscribe("/autodock_pallet/cancelDocking", 1, &Docking::cancelCb, &docking);\

    docking.status_pub_ = nodeHandle.advertise<std_msgs::UInt8>("/autodock_pallet/status", 1);
    ros::spin();

    #ifdef MOVEL_LICENSE                                                                                                    
        ml.logout();          
    #endif     

    return 0;
}
