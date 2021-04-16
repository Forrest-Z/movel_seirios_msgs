#include <ros/ros.h>
#include <DynamicLaser/LaserScanMultiFilter.h>
#include <LaserScanProfileUpdate/LaserScanProfileUpdateServer.h> 
#include <movel_hasp_vendor/license.h>

int main(int argc, char** argv)
{
    #ifdef MOVEL_LICENSE                                                                                                    
    MovelLicense ml(23);                                                                                                   
    if (!ml.login())                                                                                                      
      return 1;                                                                                                           
    #endif

    ros::init(argc, argv, "laserscan_multi_filter");

    LaserScanMultiFilter lasermultifilter("input_scan_topics", "profile1");

    ros::ServiceServer profile_update_server =  lasermultifilter.nh_.advertiseService("LaserscanMultiFilterProfileUpdate", 
                                                                                        &LaserScanProfileUpdateServer::onProfileUpdate, 
                                                                                        &lasermultifilter.laserscanprofileupdateserver);
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    ros::waitForShutdown();
    #ifdef MOVEL_LICENSE                                                                                                    
    ml.logout();          
    #endif  

    return 0;
}
