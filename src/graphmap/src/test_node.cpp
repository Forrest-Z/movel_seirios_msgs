#include <graphmap/metaplanner.hpp>
#include <tf/transform_listener.h>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "graphmap_tester");

    metaplanner::MetaPlanner mp;
    ROS_INFO("MetaPlanner construct is good");

    tf::TransformListener tfear;
    ROS_INFO("TF ear is good");
    // std::shared_ptr<costmap_2d::Costmap2DROS> nomap (new costmap_2d::Costmap2DROS("nomap", tfear));
    costmap_2d::Costmap2DROS nomap ("nomap", tfear);
    ROS_INFO("blank costmap is good");

    mp.initialize("MetaPlanner", &nomap);
    ROS_INFO("MetaPlanner init is good");

    ROS_INFO("Library is all good. Do more tests");

    return 0;
}
