#ifndef LASERSCAN_PROFILE_UPDATE_SERVER_H_
#define LASERSCAN_PROFILE_UPDATE_SERVER_H_

#include <string>
#include <sstream>
#include "yaml-cpp/yaml.h"

#include <ros/ros.h>
#include <yaml_utils/yaml_utils.h>
#include <dynamic_reconfigure/client.h>

#include <laser_filters/AngularBoundsFilterConfig.h>
#include <laser_filters/AngularBoundsFilterInPlaceConfig.h>
#include <laser_filters/BoxFilterConfig.h>
#include <laser_filters/BoxArrayFilterConfig.h>
#include <laser_filters/FootprintFilterConfig.h>
#include <laser_filters/IntensityFilterConfig.h>
#include <laser_filters/PolygonFilterConfig.h>
#include <laser_filters/RangeFilterConfig.h>
#include <laser_filters/ScanShadowsFilterConfig.h>
#include <laser_filters/SpeckleFilterConfig.h>

#include <laser_filters/ProfileUpdate.h>

typedef dynamic_reconfigure::Client<laser_filters::AngularBoundsFilterConfig> dyncfg_client_angle;
typedef dynamic_reconfigure::Client<laser_filters::AngularBoundsFilterInPlaceConfig> dyncfg_client_angleinplace;
typedef dynamic_reconfigure::Client<laser_filters::BoxFilterConfig> dyncfg_client_box;
typedef dynamic_reconfigure::Client<laser_filters::BoxArrayFilterConfig> dyncfg_client_boxarray;
typedef dynamic_reconfigure::Client<laser_filters::FootprintFilterConfig> dyncfg_client_footprint;
typedef dynamic_reconfigure::Client<laser_filters::IntensityFilterConfig> dyncfg_client_intensity;
typedef dynamic_reconfigure::Client<laser_filters::PolygonFilterConfig> dyncfg_client_polygon;
typedef dynamic_reconfigure::Client<laser_filters::RangeFilterConfig> dyncfg_client_range;
typedef dynamic_reconfigure::Client<laser_filters::ScanShadowsFilterConfig> dyncfg_client_scanshadows;
typedef dynamic_reconfigure::Client<laser_filters::SpeckleFilterConfig> dyncfg_client_speckle;

/**
 * Initialize Dynamic Reconfigure clients and contains a service call to update their configurations in real time
 */
class LaserScanProfileUpdateServer
{
public:

    void initParams(std::string multi_filter_node_name);

    /**
     * Initialize Dynamic Reconfigure clients based on the selected profile_id
     * @param profile_id profile id to be initialized with
     * @return success of obtaining config from parameter server
     */
    bool initDynClients(std::string profile_id);

    /**
     * Service call to update the laser filter configuration at runtime with different profiles
     * 
     * @param req service request contains the profile id.
     * @param res service response indicating success of profile update 
     * @return returns success of the service call
     * @see profileUpdateXML
     */
    bool onProfileUpdate(laser_filters::ProfileUpdate::Request &req, laser_filters::ProfileUpdate::Response &res);

    std::vector<std::string> input_scan_topics_;
    std::string node_name_;
    int num_scan_topics_;

private:

    /**
     * Parses through selected xmlrpcvalue and updates the dynamic reconfigure servers based on these
     * parameters. 
     * 
     * @param xml_array xmlrpcvalue config of selected profile to be updated with
     * @see onProfileUpdate
     */
    void profileUpdateXML(XmlRpc::XmlRpcValue xml_array, const int filterindex);

    /**
     * Assigns double/int value of xml to specificed variable (double will be prioritized over int)
     * 
     * @param xml xmlrpcvalue with type int or double
     * @param value variable that will be assigned to
     */
    void XMLtoDouble(XmlRpc::XmlRpcValue xml, double &value);

    /**
     * Converts polygon xmlarray type in a string to be fed to polygon dynamic reconfigure config 
     * 
     * @param polygon_xml xmlarray type fetched from the "polygon" parameter of laser_filters/LaserScanPolygonFilter
     * @return polygon parameter in string form
     */
    std::string polygonToString(const XmlRpc::XmlRpcValue& polygon_xml);

    /**
     * Dynamic Reconfigure clients
     * Note: shared ptr is used so that the client will not have the destructor called upon exiting it's current scope
     */
    // std::shared_ptr<dynamic_reconfigure::Client<laser_filters::AngularBoundsFilterConfig>> dyn_client_angle_; 
    std::vector< std::shared_ptr<dyncfg_client_angle> > clients_angle_; 
    std::vector< std::shared_ptr<dyncfg_client_angleinplace> > clients_angleinplace_; 
    std::vector< std::shared_ptr<dyncfg_client_box> > clients_box_; 
    std::vector< std::shared_ptr<dyncfg_client_boxarray> > clients_boxarray_; 
    std::vector< std::shared_ptr<dyncfg_client_footprint> > clients_footprint_; 
    std::vector< std::shared_ptr<dyncfg_client_intensity> > clients_intensity_; 
    std::vector< std::shared_ptr<dyncfg_client_polygon> > clients_polygon_; 
    std::vector< std::shared_ptr<dyncfg_client_range> > clients_range_; 
    std::vector< std::shared_ptr<dyncfg_client_scanshadows> > clients_scanshadows_; 
    std::vector< std::shared_ptr<dyncfg_client_speckle> > clients_speckle_; 
    
    /**
     * Dynamic Reconfigure configurations
     */
    laser_filters::AngularBoundsFilterConfig cfg_angle_;
    laser_filters::AngularBoundsFilterInPlaceConfig cfg_angleinplace_;
    laser_filters::BoxFilterConfig cfg_box_;
    laser_filters::BoxArrayFilterConfig cfg_boxarray_;
    laser_filters::FootprintFilterConfig cfg_footprint_;
    laser_filters::IntensityFilterConfig cfg_intensity_;
    laser_filters::PolygonFilterConfig cfg_polygon_;
    laser_filters::RangeFilterConfig cfg_range_;
    laser_filters::ScanShadowsFilterConfig cfg_scanshadows_;
    laser_filters::SpeckleFilterConfig cfg_speckle_; 
};

void LaserScanProfileUpdateServer::initParams(std::string multi_filter_node_name)
{
    node_name_ = multi_filter_node_name;
    std::string laserscan_topics;
    ros::param::param<std::string>("/" + node_name_ + "/laserscan_topics", laserscan_topics, "");

    //seperate the string using space " "
    std::istringstream iss(laserscan_topics);
	copy(std::istream_iterator<std::string>(iss), 
         std::istream_iterator<std::string>(), 
         std::back_inserter<std::vector<std::string> >(input_scan_topics_));

    num_scan_topics_ = input_scan_topics_.size();
    for (int i = 0; i < input_scan_topics_.size(); i++)
    {
        std::string topic_modified = input_scan_topics_[i].substr(1, input_scan_topics_[i].size());
        input_scan_topics_[i] = topic_modified;
        // ROS_INFO("Topics added: %s", topic_modified.c_str());
    }

    //resize all the dynamic reconfigure client vectors
    clients_angle_.resize(num_scan_topics_);
    clients_angleinplace_.resize(num_scan_topics_);
    clients_box_.resize(num_scan_topics_); 
    clients_boxarray_.resize(num_scan_topics_); 
    clients_footprint_.resize(num_scan_topics_); 
    clients_intensity_.resize(num_scan_topics_); 
    clients_polygon_.resize(num_scan_topics_); 
    clients_range_.resize(num_scan_topics_); 
    clients_scanshadows_.resize(num_scan_topics_); 
    clients_speckle_.resize(num_scan_topics_); 
}

bool LaserScanProfileUpdateServer::initDynClients(std::string profile_id)
{
    XmlRpc::XmlRpcValue config_xml; 
    std::string dyncfg_server_name, filter_type, filter_name;

    if (ros::param::get(node_name_, config_xml))
    {
        YAML::Node config = parseStruct(config_xml, node_name_);

        //iterate through each scan topic
        for (int i=0; i < num_scan_topics_; i++) 
        {
            //iterate through each filter profile
            for (int j = 0; j < config[input_scan_topics_[i]][profile_id]["scan_filter_chain"].size(); j++) 
            {
                filter_type = config[input_scan_topics_[i]][profile_id]["scan_filter_chain"][j]["type"].as<std::string>();
                filter_name = config[input_scan_topics_[i]][profile_id]["scan_filter_chain"][j]["name"].as<std::string>();
                
                dyncfg_server_name =   node_name_ + "/" + filter_name;  // should be in the form "/node_name/filter_name"

                if (filter_type == "laser_filters/LaserScanAngularBoundsFilter"){
                    std::shared_ptr<dyncfg_client_angle> client_angle(new dyncfg_client_angle(dyncfg_server_name));
                    clients_angle_[i] = client_angle;
                }
                else if (filter_type == "laser_filters/LaserScanAngularBoundsFilterInPlace"){
                    std::shared_ptr<dyncfg_client_angleinplace> client_angleinplace(new dyncfg_client_angleinplace(dyncfg_server_name));
                    clients_angleinplace_[i] = client_angleinplace;
                }
                else if (filter_type == "laser_filters/LaserScanBoxFilter"){
                    std::shared_ptr<dyncfg_client_box> client_box(new dyncfg_client_box(dyncfg_server_name));
                    clients_box_[i] = client_box;
                }
               else if (filter_type == "laser_filters/LaserScanBoxArrayFilter"){
                    std::shared_ptr<dyncfg_client_boxarray> client_boxarray(new dyncfg_client_boxarray(dyncfg_server_name));
                    clients_boxarray_[i] = client_boxarray;
                }
                else if (filter_type == "laser_filters/LaserScanFootprintFilter"){
                    std::shared_ptr<dyncfg_client_footprint> client_footprint(new dyncfg_client_footprint(dyncfg_server_name));
                    clients_footprint_[i] = client_footprint;
                }
                else if (filter_type == "laser_filters/LaserScanIntensityFilter"){
                    std::shared_ptr<dyncfg_client_intensity> client_intensity(new dyncfg_client_intensity(dyncfg_server_name));
                    clients_intensity_[i] = client_intensity;
                }
                else if (filter_type == "laser_filters/LaserScanPolygonFilter"){
                    std::shared_ptr<dyncfg_client_polygon> client_polygon(new dyncfg_client_polygon(dyncfg_server_name));
                    clients_polygon_[i] = client_polygon;
                }
                else if (filter_type == "laser_filters/LaserScanRangeFilter"){
                    std::shared_ptr<dyncfg_client_range> client_range(new dyncfg_client_range(dyncfg_server_name));
                    clients_range_[i] = client_range;
                }
                else if (filter_type == "laser_filters/ScanShadowsFilter"){
                    std::shared_ptr<dyncfg_client_scanshadows> client_scanshadows(new dyncfg_client_scanshadows(dyncfg_server_name));
                    clients_scanshadows_[i] = client_scanshadows;
                }
                else if (filter_type == "laser_filters/LaserScanSpeckleFilter"){
                    std::shared_ptr<dyncfg_client_speckle> client_speckle(new dyncfg_client_speckle(dyncfg_server_name));
                    clients_speckle_[i] = client_speckle;
                }
                ROS_INFO("DYNAMIC LASER CLIENT: Added client: %s", dyncfg_server_name.c_str());
            }
        }
    }
    else {
        return false;
    }
    return true;
}

bool LaserScanProfileUpdateServer::onProfileUpdate(laser_filters::ProfileUpdate::Request &req, laser_filters::ProfileUpdate::Response &res)
{
    std::string profileid = req.profileid;
    std::string scantopic = req.scantopic;
    XmlRpc::XmlRpcValue xml_array; 
    int filterindex;

    std::string parampath = "/" + node_name_ + "/"  + scantopic + "/" + profileid + "/scan_filter_chain/";

    //obtain the filter index in order to update the appropriate client
    for (int i=0; i < num_scan_topics_; i++)
    {
        if (scantopic.compare(input_scan_topics_[i]) == 0 )
            filterindex = i;
    }

    if(ros::param::getCached(parampath, xml_array)) 
    {
        profileUpdateXML(xml_array, filterindex); 
        res.success = true;
        ROS_INFO("Profile update successful!");
        return true;
    }
    else 
    {
        res.success = false;
        ROS_INFO("Profile update failed, please check your config file for missing fields.");
        return true;
    }
}

void LaserScanProfileUpdateServer::profileUpdateXML(XmlRpc::XmlRpcValue xml_array, const int filterindex)
{
    for(int i=0; i <xml_array.size(); i++) //iterate through number of filtertypes in each profile
    {
        std::string filter_type = static_cast<std::string>(xml_array[i]["type"]);
        std::string filter_name = static_cast<std::string>(xml_array[i]["name"]);

        if(filter_type == "laser_filters/LaserScanAngularBoundsFilter") 
        {
            XMLtoDouble(xml_array[i]["params"]["lower_angle"], cfg_angle_.lower_angle );
            XMLtoDouble(xml_array[i]["params"]["upper_angle"], cfg_angle_.upper_angle );
            cfg_angle_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            clients_angle_[filterindex]->setConfiguration(cfg_angle_);
        }
        else if (filter_type == "laser_filters/LaserScanAngularBoundsFilterInPlace")
        {
            XMLtoDouble(xml_array[i]["params"]["lower_angle"], cfg_angleinplace_.lower_angle );
            XMLtoDouble(xml_array[i]["params"]["upper_angle"], cfg_angleinplace_.upper_angle );
            cfg_angleinplace_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            clients_angleinplace_[filterindex]->setConfiguration(cfg_angleinplace_);
        }
        else if(filter_type == "laser_filters/LaserScanBoxFilter") 
        {
            XMLtoDouble(xml_array[i]["params"]["max_x"], cfg_box_.max_x);
            XMLtoDouble(xml_array[i]["params"]["max_y"], cfg_box_.max_y);
            XMLtoDouble(xml_array[i]["params"]["max_z"], cfg_box_.max_z);
            XMLtoDouble(xml_array[i]["params"]["min_x"], cfg_box_.min_x);
            XMLtoDouble(xml_array[i]["params"]["min_y"], cfg_box_.min_y);
            XMLtoDouble(xml_array[i]["params"]["min_z"], cfg_box_.min_z);
            cfg_box_.invert = static_cast<bool>(xml_array[i]["params"]["invert"]);
            cfg_box_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            clients_box_[filterindex]->setConfiguration(cfg_box_);
        }

        else if(filter_type == "laser_filters/LaserScanBoxArrayFilter") 
        {
            cfg_boxarray_.box_array = polygonToString(xml_array[i]["params"]["box_array"]);
            cfg_boxarray_.invert = static_cast<bool>(xml_array[i]["params"]["invert"]);
            cfg_boxarray_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            clients_boxarray_[filterindex]->setConfiguration(cfg_boxarray_);
        }

        else if ( filter_type == "laser_filters/LaserScanFootprintFilter")
        {
            XMLtoDouble(xml_array[i]["params"]["inscribed_radius"], cfg_footprint_.inscribed_radius);
            cfg_footprint_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            clients_footprint_[filterindex]->setConfiguration(cfg_footprint_);
        }
        else if ( filter_type == "laser_filters/LaserScanIntensityFilter")
        {
            XMLtoDouble(xml_array[i]["params"]["lower_threshold"], cfg_intensity_.lower_threshold);
            XMLtoDouble(xml_array[i]["params"]["upper_threshold"], cfg_intensity_.upper_threshold);
            cfg_intensity_.invert = static_cast<bool>(xml_array[i]["params"]["invert"]);
            cfg_intensity_.filter_override_range = static_cast<bool>(xml_array[i]["params"]["filter_override_range"]);
            cfg_intensity_.filter_override_intensity = static_cast<bool>(xml_array[i]["params"]["filter_override_intensity"]);
            cfg_intensity_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            clients_intensity_[filterindex]->setConfiguration(cfg_intensity_);
        }
        else if ( filter_type == "laser_filters/LaserScanPolygonFilter")
        {
            cfg_polygon_.polygon = polygonToString(xml_array[i]["params"]["polygon"]);
            XMLtoDouble(xml_array[i]["params"]["polygon_padding"], cfg_polygon_.polygon_padding);
            cfg_polygon_.invert = static_cast<bool>(xml_array[i]["params"]["invert"]);
            cfg_polygon_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            clients_polygon_[filterindex]->setConfiguration(cfg_polygon_);
        }    
        else if ( filter_type == "laser_filters/LaserScanRangeFilter")
        {
            cfg_range_.use_message_range_limits = static_cast<bool>(xml_array[i]["params"]["use_message_range_limits"]);
            XMLtoDouble(xml_array[i]["params"]["lower_threshold"], cfg_range_.lower_threshold);
            XMLtoDouble(xml_array[i]["params"]["upper_threshold"], cfg_range_.upper_threshold);
            XMLtoDouble(xml_array[i]["params"]["lower_replacement_value"], cfg_range_.lower_replacement_value);
            XMLtoDouble(xml_array[i]["params"]["upper_replacement_value"], cfg_range_.upper_replacement_value);
            cfg_range_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            clients_range_[filterindex]->setConfiguration(cfg_range_);
        }
        else if ( filter_type == "laser_filters/ScanShadowsFilter")
        {
            XMLtoDouble(xml_array[i]["params"]["min_angle"], cfg_scanshadows_.min_angle );
            XMLtoDouble(xml_array[i]["params"]["max_angle"], cfg_scanshadows_.max_angle );
            cfg_scanshadows_.window = static_cast<int>(xml_array[i]["params"]["window"]);
            cfg_scanshadows_.neighbors = static_cast<int>(xml_array[i]["params"]["neighbors"]);
            cfg_scanshadows_.remove_shadow_start_point = static_cast<bool>(xml_array[i]["params"]["remove_shadow_start_point"]);
            cfg_scanshadows_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            clients_scanshadows_[filterindex]->setConfiguration(cfg_scanshadows_);
        }      
        else if ( filter_type  == "laser_filters/LaserScanSpeckleFilter")
        {
            cfg_speckle_.filter_type = static_cast<int>(xml_array[i]["params"]["filter_type"]);
            XMLtoDouble(xml_array[i]["params"]["max_angle"], cfg_speckle_.max_range );
            XMLtoDouble(xml_array[i]["params"]["max_range_difference"], cfg_speckle_.max_range_difference );
            cfg_speckle_.filter_window = static_cast<int>(xml_array[i]["params"]["filter_window"]);
            cfg_speckle_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            clients_speckle_[filterindex]->setConfiguration(cfg_speckle_);
        }       
    }
}

void LaserScanProfileUpdateServer::XMLtoDouble(XmlRpc::XmlRpcValue xml, double &value)
{
    if (xml.getType() == XmlRpc::XmlRpcValue::TypeDouble){
        value = static_cast<double>(xml);
    }
    else{
        value = static_cast<int>(xml);
    }
}

std::string LaserScanProfileUpdateServer::polygonToString(const XmlRpc::XmlRpcValue& polygon_xml)
{
    std::string polygon_string = "[";
    bool first = true;
    double x;
    double y;
    for (int i = 0; i < polygon_xml.size(); ++i)
    {
        XmlRpc::XmlRpcValue point = polygon_xml[i];
        if (!first)
        {
            polygon_string += ",";
        }
        first = false;
        XMLtoDouble(point[0], x);
        XMLtoDouble(point[1], y);
        polygon_string += "[" + std::to_string(x) + ", " + std::to_string(y) + "]";

    }
    polygon_string += "]";
    return polygon_string;
}

#endif