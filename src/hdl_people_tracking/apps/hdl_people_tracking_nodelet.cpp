#include <mutex>
#include <memory>
#include <iostream>
#include <boost/format.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <pcl/filters/filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <movel_seirios_msgs/ClearTarget.h>
#include <visualization_msgs/MarkerArray.h>
#include <movel_seirios_msgs/Track.h>
#include <movel_seirios_msgs/TrackArray.h>
#include <movel_seirios_msgs/ClusterArray.h>
#include <movel_seirios_msgs/Reset.h>
#include <kkl/cvk/cvutils.hpp>
#include <hdl_people_tracking/people_tracker.hpp>
#include "std_msgs/Int32.h"

namespace hdl_people_tracking {

class HdlPeopleTrackingNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  HdlPeopleTrackingNodelet() {}
  virtual ~HdlPeopleTrackingNodelet() {}

  void onInit() override {
    nh_ = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    tracker_.reset(new PeopleTracker(private_nh));
    color_palette = cvk::create_color_palette(16);

    tracks_pub_ = private_nh.advertise<movel_seirios_msgs::TrackArray>("tracks", 10);
    marker_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("markers", 10);
    clusters_sub_ = nh_.subscribe("/hdl_people_detection_nodelet/clusters", 1, &HdlPeopleTrackingNodelet::callback, this);
    target_sub_ = nh_.subscribe("/velocity_commands/target_id", 1, &HdlPeopleTrackingNodelet::cb, this);
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &HdlPeopleTrackingNodelet::cmdVelCb, this);
    //Service
    reset_ = nh_.advertiseService("/hdl_people_tracking/reset", &HdlPeopleTrackingNodelet::reset, this);
    reset_client_ = nh_.serviceClient<movel_seirios_msgs::ClearTarget>("/hdl_people_tracking/clear_target", true);
  }

private:
  void cb (const std_msgs::Int32 &msg){
      target_id_ = int(msg.data);
  }
  
  void cmdVelCb (const geometry_msgs::Twist &msg){
      linear_x_ = msg.linear.x;
      angular_z_ = msg.angular.z;
  }

  void callback(const movel_seirios_msgs::ClusterArrayPtr& clusters_msg) {
    // remove non-human detections
    auto remove_loc = std::remove_if(clusters_msg->clusters.begin(), clusters_msg->clusters.end(), [=](const movel_seirios_msgs::Cluster& cluster) { return !cluster.is_human; });
    clusters_msg->clusters.erase(remove_loc, clusters_msg->clusters.end());

    // update people tracker
    tracker_->predict(clusters_msg->header.stamp, linear_x_, angular_z_);
    tracker_->correct(clusters_msg->header.stamp, clusters_msg->clusters, target_id_);

    // publish tracks msg
    //if(tracks_pub.getNumSubscribers()) {
      tracks_pub_.publish(create_tracks_msg(clusters_msg->header));
    //}

    // publish rviz markers
    //if(marker_pub.getNumSubscribers()) {
      marker_pub_.publish(create_tracked_people_marker(clusters_msg->header));
    //}
  }
  
  bool reset(movel_seirios_msgs::Reset::Request &req, movel_seirios_msgs::Reset::Response &res){
    if (req.restart){
      tracker_.reset(new PeopleTracker(private_nh));
      movel_seirios_msgs::ClearTarget srv;
      srv.request.clear_target = true;
      try {
       reset_client_.call(srv);
       res.restarted = true;
       return true;
      } 
      catch (...)
     {
      ROS_ERROR("Failed to call service /hdl_people_tracking/clear_target");
      res.restarted = false;
      return false;
     }
      
    }
    else{
      res.restarted = false;
      return false;
    }
  }


    movel_seirios_msgs::TrackArrayConstPtr create_tracks_msg(const std_msgs::Header& header) const {
    movel_seirios_msgs::TrackArrayPtr tracks_msg(new movel_seirios_msgs::TrackArray());
    tracks_msg->header = header;
    tracks_msg->tracks.resize(tracker_->people.size());
    for(int i=0; i<tracker_->people.size(); i++) {
      const auto& track = tracker_->people[i];
      auto& track_msg = tracks_msg->tracks[i];

      track_msg.id = track->id();
      track_msg.age = (track->age(header.stamp)).toSec();
      track_msg.pos.x = track->observed_position().x();
      track_msg.pos.y = track->observed_position().y();
      track_msg.pos.z = track->observed_position().z();
      track_msg.vel.x = track->velocity().x();
      track_msg.vel.y = track->velocity().y();
      track_msg.vel.z = track->velocity().z();

      Eigen::Matrix3d pos_cov = track->positionCov();
      for(int k=0; k<3; k++) {
        for(int j=0; j<3; j++) {
          track_msg.pos_cov[k*3 + j] = pos_cov(k, j);
        }
      }

      Eigen::Matrix3d vel_cov = track->velocityCov();
      for(int k=0; k<3; k++) {
        for(int j=0; j<3; j++) {
          track_msg.vel_cov[k*3 + j] = vel_cov(k, j);
        }
      }

      const movel_seirios_msgs::Cluster* associated = boost::any_cast<movel_seirios_msgs::Cluster>(&track->lastAssociated());
      if(!associated) {
        continue;
      }

      track_msg.associated.resize(1);
      track_msg.associated[0] = (*associated);
    }

    return tracks_msg;
  }

  visualization_msgs::MarkerArrayConstPtr create_tracked_people_marker(const std_msgs::Header& header) const {
    visualization_msgs::MarkerArrayPtr markers_ptr(new visualization_msgs::MarkerArray());

    visualization_msgs::MarkerArray& markers = *markers_ptr;
    markers.markers.reserve(tracker_->people.size() + 1);
    markers.markers.resize(1);

    visualization_msgs::Marker& boxes = markers.markers[0];
    boxes.header = header;
    boxes.action = visualization_msgs::Marker::ADD;
    boxes.lifetime = ros::Duration(1.0);

    boxes.ns = "boxes";
    boxes.type = visualization_msgs::Marker::CUBE_LIST;
    boxes.colors.reserve(tracker_->people.size());
    boxes.points.reserve(tracker_->people.size());

    boxes.pose.position.z = 0.0f;
    boxes.pose.orientation.w = 1.0f;

    boxes.scale.x = 0.5;
    boxes.scale.y = 0.5;
    boxes.scale.z = 1.2;

    for(int i=0; i<tracker_->people.size(); i++) {
      const auto& person = tracker_->people[i];
      const auto& color = color_palette[person->id() % color_palette.size()];

      if(person->correctionCount() < 5) {
        continue;
      }

      std_msgs::ColorRGBA rgba;
      rgba.r = color[2] / 255.0;
      rgba.g = color[1] / 255.0;
      rgba.b = color[0] / 255.0;
      rgba.a = 0.6f;
      markers.markers[0].colors.push_back(rgba);

      geometry_msgs::Point point;
      point.x = person->position().x();
      point.y = person->position().y();
      point.z = person->position().z();
      markers.markers[0].points.push_back(point);

      visualization_msgs::Marker text;
      text.header = markers.markers[0].header;
      text.action = visualization_msgs::Marker::ADD;
      text.lifetime = ros::Duration(1.0);

      text.ns = (boost::format("text%d") % person->id()).str();
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.scale.z = 0.5;

      text.pose.position = point;
      text.pose.position.z += 0.7;
      text.color.r = text.color.b = text.color.a = 1.0;
      text.color.g = 250.0;
      text.text = (boost::format("id:%d") % person->id()).str();

      markers.markers.push_back(text);
    }

    return markers_ptr;
  }

private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Publisher tracks_pub_;
  ros::Publisher marker_pub_;
  ros::Subscriber clusters_sub_;
  ros::Subscriber target_sub_;
  ros::Subscriber cmd_vel_sub_;
  boost::circular_buffer<cv::Scalar> color_palette;
  float linear_x_, angular_z_;
  std::unique_ptr<PeopleTracker> tracker_;
  int target_id_;
  ros::ServiceServer reset_;
  ros::ServiceClient reset_client_; //to reset target and cancel goal
};

}


PLUGINLIB_EXPORT_CLASS(hdl_people_tracking::HdlPeopleTrackingNodelet, nodelet::Nodelet)
