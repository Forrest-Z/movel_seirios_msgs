#include "ros/ros.h"
#include <laser_filters/ProfileUpdate.h>
#include <string>
#include <movel_hasp_vendor/license.h>

int main(int argc, char **argv)
{
    #ifdef MOVEL_LICENSE                                                                                                    
    MovelLicense ml(23);                                                                                                   
    if (!ml.login())                                                                                                      
      return 1;                                                                                                           
    #endif

    ros::init(argc, argv, "Profile_Update_Client");
    if (argc != 3)
    {
        ROS_INFO("Usage: Enter a scan topic and profile ID to retrieve it's parameters");
	#ifdef MOVEL_LICENSE
        ml.logout();
        #endif
        return 1;        
    }
    
    ros::NodeHandle nh;
    ros::ServiceClient client_update_profile = nh.serviceClient<laser_filters::ProfileUpdate>("LaserscanMultiFilterProfileUpdate");
    laser_filters::ProfileUpdate profile_update_srv;

    profile_update_srv.request.scantopic = (argv[1]);
    profile_update_srv.request.profileid = (argv[2]);                
    
    if(client_update_profile.call(profile_update_srv)){
        if (profile_update_srv.response.success){
            ROS_INFO("Successfully reconfigured scantopic (%s) to profileID  (%s)", (profile_update_srv.request.scantopic).c_str(), (profile_update_srv.request.profileid).c_str() );
            #ifdef MOVEL_LICENSE                                                                                                    
            ml.logout();
            #endif
	    return 0;
        }
        else{
            ROS_ERROR("Either scantopic (%s) or profile (%s) not found, please try with other inputs", (profile_update_srv.request.scantopic).c_str(), (profile_update_srv.request.profileid).c_str());
	    #ifdef MOVEL_LICENSE
            ml.logout();
            #endif
            return 1;
        }
    }
    else{
        ROS_ERROR("Failed to connect to retrieve service!");
	#ifdef MOVEL_LICENSE
        ml.logout();
        #endif
        return 1;

    }
}
