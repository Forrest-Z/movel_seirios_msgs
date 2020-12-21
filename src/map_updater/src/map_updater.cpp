#include <ros/ros.h>
#include "map_updater/map_updater.h"

/**
 * @brief constructor for map updater
 */
MapUpdater::MapUpdater()
{
    ros::NodeHandle nh;

    // Read parameters
    readParams();

    // Publishers, Subscribers, Clients, Servers
    pub_updated_map = nh.advertise<map_msgs::OccupancyGridUpdate>("map_updates",100);
    client_map = nh.serviceClient<nav_msgs::GetMap>("static_map");
    service_map_restore = nh.advertiseService("restore_map", &MapUpdater::restoreMap,this);
    sub_global_map = nh.subscribe("move_base/global_costmap/costmap", 30, &MapUpdater::globalMapCallback, this);
    sub_local_map = nh.subscribe("move_base/local_costmap/costmap", 30, &MapUpdater::localMapCallback, this);
}

/**
 * @brief Restore the updated map with the static map
 * @param req
 * @param res
 */
bool MapUpdater::restoreMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    // Overwrite the map with prior static map
    updatemap.header = globalmap.header;
    updatemap.x = 0;
    updatemap.y = 0;
    updatemap.width = staticmap.info.width;
    updatemap.height = staticmap.info.height;
    updatemap.data = staticmap.data;
    dynamicmap.data = staticmap.data;

    pub_updated_map.publish(updatemap);     //publish

    return true;
}

/**
 * @brief Callback for the global costmap topic
 * @param globalmap_msg
 */
void MapUpdater::globalMapCallback(const nav_msgs::OccupancyGrid& globalmap_msg)
{
    // std::cout<<"Global callback"<<std::endl;
    globalmap = globalmap_msg;
}

/**
 * @brief Callback for the local costmap topic
 * @param localmap_msg
 */
void MapUpdater::localMapCallback(const nav_msgs::OccupancyGrid& localmap_msg)
{
    // std::cout<<"Local callback"<<std::endl;
    localmap = localmap_msg;

    // Get the static map
    nav_msgs::GetMap srv;
    if ( client_map.call(srv) && (initial_ == true || map_once_ == false))
    {
        dynamicmap = srv.response.map;
        initial_ = false;
        staticmap = dynamicmap;
    }
    updateMap();
}

/**
 * @brief Procedure for updating the map
 */
void MapUpdater::updateMap()
{
    // std::cout<<globalmap.info.width<<dynamicmap.info.width<<std::endl;
    
    // When all the data have been collected
    if((globalmap.info.width != 0) && (dynamicmap.info.width != 0))
    {                
        trim_width = localmap.info.width - (update_width_ / localmap.info.resolution);    //trim for local costmap
        trim_height = localmap.info.height - (update_height_ / localmap.info.resolution);
        dx = (globalmap.info.origin.position.x - localmap.info.origin.position.x);      //difference from global and local costmap
        dy = (globalmap.info.origin.position.y - localmap.info.origin.position.y);
        if(dx < 0)
            dx = dx * -1;
        if(dy < 0)
            dy = dy * -1;

        // Define the starting index
        dx += ((localmap.info.width * localmap.info.resolution) - update_width_)/2;
        dy += ((localmap.info.width * localmap.info.resolution) - update_height_)/2;
        nx = dx / localmap.info.resolution;
        ny = dy / localmap.info.resolution;
        id_start = globalmap.info.width * ny + nx;
        new_width = localmap.info.width - trim_width;
        new_height = localmap.info.height - trim_height;

        // Updating the message
        updatemap.header = localmap.header;
        updatemap.x = nx;
        updatemap.y = ny;
        updatemap.width = new_width;
        updatemap.height = new_height;
        updatemap.data = dynamicmap.data;

        // Update process
        x_trav = -1;
        y_trav = 0;
        for(int i = 0;i<new_width*new_height;i++)
        {
            x_trav++;
            if (x_trav == new_width)
            {
                y_trav++;
                x_trav = 0;
            }

            // Get the value at this index
            pro_loc = localmap.data[int(x_trav + (trim_width/2) + ((y_trav + (trim_height/2)) * localmap.info.width))];
            pro_glo = globalmap.data[int(id_start + x_trav + (y_trav * globalmap.info.width))];
            pro_sta =  dynamicmap.data[id_start + x_trav + (y_trav * globalmap.info.width)];
            
            // Initialize the map
            updatemap.data[x_trav + (y_trav * new_width)] = pro_sta;

            // If the map is unoccupied and the local costmap doesn't exsist
            if( (pro_loc != pro_glo) && (pro_loc < loc_th_) && (pro_glo > glo_th_ ) && (pro_sta == 100))
            {
                std::cout<<"Updated"<<std::endl;
                updatemap.data[x_trav + (y_trav * new_width)] = 0;
                dynamicmap.data[id_start + x_trav + (y_trav * globalmap.info.width)] = 0;       //update the temp
            }
        }
        pub_updated_map.publish(updatemap);
    }
    else
    {
        ROS_INFO("The data haven't completed yet!");
    }
}

/**
 * @brief Read parameters from yaml file
 */
void MapUpdater::readParams()
{
    ros::NodeHandle nh_("~");
    nh_.getParam("subscribe_only_once", map_once_);
    nh_.getParam("local_threshold", loc_th_);
    nh_.getParam("global_threshold", glo_th_);
    nh_.getParam("width_update", update_width_);
    nh_.getParam("height_update", update_height_);

}
