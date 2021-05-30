#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include "multi_map_server/LoadMap.h"
#include <movel_hasp_vendor/license.h>
using namespace std;


int main(int argc, char** argv){

  #ifdef MOVEL_LICENSE
	  MovelLicense ml(2);
	  if (!ml.login())
	    return 1;
  #endif

  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;
  ros::NodeHandle node_priv("~");

  tf::TransformListener listener;

  ros::Rate rate(10.0);


  ros::ServiceClient swapper_client = node.serviceClient<multi_map_server::LoadMap>("load_map");
  multi_map_server::LoadMap srv;
  
  XmlRpc::XmlRpcValue my_list;
  node_priv.getParam("rooms", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  std::map<int,std::string> room_map;
  std::map<std::vector<std::vector<double>>,int > points_map;

  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
  	std::string location;int id ;
  	XmlRpc::XmlRpcValue points;
  	XmlRpc::XmlRpcValue sublist = my_list[i];
    id = sublist["id"]; 
    points = (sublist["pos"]);
    // std::cout<<points.size()<<std::endl;
    location = static_cast<std::string>(sublist["map_location"]);
    vector<vector<double>> region;
    for (int j=0;j<points.size();j++){
      // std::cout<< points[j].size()<<endl;
      // std::cout<<points[j][0][0]<<","<<points[j][0][1]<<","<<points[j][1][0]<<","<<points[j][1][1]<<std::endl;
      ROS_ASSERT(points[j].size() == 2); //check for two points top points(x,y) and bottom points(x,y) // Throw error points exceed two
      for (int x=0 ;x < points[j].size();x++){
        ROS_ASSERT(points[j][x].size() == 2); //check the array should contain 2 points (x,y) , Throw error if condition fails
      }
    	std::vector<double> pt;
    	pt.push_back(static_cast<double>(points[j][0][0]));
      pt.push_back(static_cast<double>(points[j][0][1]));
      region.push_back(pt);
      pt.clear();
      pt.push_back(static_cast<double>(points[j][1][0]));
      pt.push_back(static_cast<double>(points[j][1][1]));
      region.push_back(pt);
      points_map[region] = id;
      region.clear(); 
    }
    room_map[id] = location ;
   }
  

  int swap_id =0 ,old_swap =0;
  
  while (node.ok()){

    tf::StampedTransform transform;
    try { 
      
      listener.waitForTransform( "map","base_footprint", ros::Time(0), ros::Duration(10.0) );
      listener.lookupTransform("map","base_footprint",  ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

   


     map< std::vector<std::vector<double>>,int>::iterator itr;
     for (itr = points_map.begin(); itr != points_map.end(); ++itr) { 
        // cout << "id :\t" << itr->second <<" \n ";
        std::vector<std::vector<double>> validator;
        validator = itr->first;

        //to find the limit
        double smallx = (validator[0][0] > validator[1][0]) ? validator[1][0] : validator[0][0];
        double largex = (validator[0][0] < validator[1][0]) ? validator[1][0] : validator[0][0];
        double smally = (validator[0][1] > validator[1][1]) ? validator[1][1] : validator[0][1];
        double largey = (validator[0][1] < validator[1][1]) ? validator[1][1] : validator[0][1];
        
        if(transform.getOrigin().x() < largex && transform.getOrigin().x() > smallx &&
         transform.getOrigin().y() < largey && transform.getOrigin().y() > smally){

            swap_id = itr->second;
           }
        }

	if(old_swap != swap_id){
		//service call 
		old_swap = swap_id;
	
			ROS_INFO(" entered swapping");


			srv.request.map_url = room_map[swap_id];
			if (swapper_client.call(srv))
			  { ROS_INFO("Result: %s", srv.response.result.c_str());
			  }
			  else{
			    ROS_ERROR("Failed to call service add_two_ints");
			    return 1;
			  }
		
	}

    rate.sleep();
  }
  return 0;
  #ifdef MOVEL_LICENSE
	  ml.logout();
  #endif

};
