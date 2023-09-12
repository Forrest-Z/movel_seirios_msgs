#include <DynamicLaser/dynamic_laser_client.h>
#include <movel_hasp_vendor/license.h>

DynLaser dynlaser;

int main(int argc, char **argv)
{ 
	#ifdef MOVEL_LICENSE
        MovelLicense ml;
        if (!ml.login())
            return 1;
        #endif

	ros::init (argc, argv, "dynamic_laser_client_node"); //must be same name as that in launch file
	ros::NodeHandle nh_private_("~");

	dynlaser.node_name_ = ros::this_node::getName() + "/";

	if(dynlaser.initDynClients("init")){
		ROS_INFO("Parameter loading succeeded"); 
	}
	else 
	{
		ROS_ERROR("Parameter loading failed"); 
		return 1;
	}

	ros::ServiceServer profile_update = nh_private_.advertiseService("profile_update", &DynLaser::onProfileUpdate, &dynlaser);
	
	ros::spin();
	#ifdef MOVEL_LICENSE
        ml.logout();
        #endif
	return 0;
}

