#include "movel_task_duration_estimator/task_duration_estimator.h"

int main(int argc, char * argv[])
{
    ros::init(argc,argv,"task_duration_estimator");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate rate(100);

    TaskDurationEstimator task_duration_estimator(nh, private_nh);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
