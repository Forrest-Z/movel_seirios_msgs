#include <ros/ros.h>
#include <string>
#include <cmath>

#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

class ObstClass{
    private:
    // ROS params

    // Variables
    string name_;
    float peripheral_angle_;
    float detect_range_;


    public:
    ros::NodeHandle n_;
    ros::Subscriber laser_scan_sub_;
    ros::Publisher obst_pub_;

    ObstClass(ros::NodeHandle* nodehandle);
    template <typename param_type>
    bool load_param_util(std::string param_name, param_type& output);
    bool loadParams();

    void laserScanCB(const sensor_msgs::LaserScan::ConstPtr& );


};