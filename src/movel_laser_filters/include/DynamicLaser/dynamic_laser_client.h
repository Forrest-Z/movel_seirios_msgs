#ifndef DYNAMIC_LASER_CLIENT_H_
#define DYNAMIC_LASER_CLIENT_H_

#include <ros/ros.h>
#include "yaml-cpp/yaml.h"
#include <ros_utils/ros_utils.h>
#include <yaml_utils/yaml_utils.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <dynamic_reconfigure/client.h>
#include <string>
#include <sstream>
#include <movel_laser_filters/AngularBoundsFilterConfig.h>
#include <movel_laser_filters/AngularBoundsFilterInPlaceConfig.h>
#include <movel_laser_filters/BoxFilterConfig.h>
#include <movel_laser_filters/FootprintFilterConfig.h>
#include <movel_laser_filters/IntensityFilterConfig.h>
#include <movel_laser_filters/PolygonFilterConfig.h>
#include <movel_laser_filters/RangeFilterConfig.h>
#include <movel_laser_filters/ScanShadowsFilterConfig.h>
#include <movel_laser_filters/SpeckleFilterConfig.h>
#include <movel_laser_filters/ProfileUpdate.h>

/**
 * Initialize Dynamic Reconfigure clients and contains a service call to update their configurations in real time
 */
class DynLaser
{
public:
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
     * @see profileUpdate
     */
    bool onProfileUpdate(movel_laser_filters::ProfileUpdate::Request &req, movel_laser_filters::ProfileUpdate::Response &res);

    std::string node_name_;
private:

    /**
     * Parses through selected xmlrpcvalue and updates the dynamic reconfigure servers based on these
     * parameters. 
     * 
     * @param xml_array xmlrpcvalue config of selected profile to be updated with
     * @see onProfileUpdate
     */
    void profileUpdateXML(XmlRpc::XmlRpcValue xml_array);

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
     * @param polygon_xml xmlarray type fetched from the "polygon" parameter of movel_laser_filters/LaserScanPolygonFilter
     * @return polygon parameter in string form
     */
    std::string polygonToString(const XmlRpc::XmlRpcValue& polygon_xml);

    /**
     * Dynamic Reconfigure clients
     */
    std::shared_ptr<dynamic_reconfigure::Client<movel_laser_filters::AngularBoundsFilterConfig>> dyn_client_angle_; //shared ptr is used so that the client will not have the destructor called upon exiting it's current scope
    std::shared_ptr<dynamic_reconfigure::Client<movel_laser_filters::AngularBoundsFilterInPlaceConfig>> dyn_client_angleinplace_;
    std::shared_ptr<dynamic_reconfigure::Client<movel_laser_filters::BoxFilterConfig>> dyn_client_box_;
    std::shared_ptr<dynamic_reconfigure::Client<movel_laser_filters::FootprintFilterConfig>> dyn_client_footprint_;
    std::shared_ptr<dynamic_reconfigure::Client<movel_laser_filters::IntensityFilterConfig>> dyn_client_intensity_;
    std::shared_ptr<dynamic_reconfigure::Client<movel_laser_filters::PolygonFilterConfig>> dyn_client_polygon_;
    std::shared_ptr<dynamic_reconfigure::Client<movel_laser_filters::RangeFilterConfig>> dyn_client_range_;
    std::shared_ptr<dynamic_reconfigure::Client<movel_laser_filters::ScanShadowsFilterConfig>> dyn_client_shadow_;
    std::shared_ptr<dynamic_reconfigure::Client<movel_laser_filters::SpeckleFilterConfig>> dyn_client_speckle_;

    /**
     * Dynamic Reconfigure configurations
     */
    movel_laser_filters::AngularBoundsFilterConfig anglecfg_;
    movel_laser_filters::AngularBoundsFilterInPlaceConfig angleinplacecfg_;
    movel_laser_filters::BoxFilterConfig boxcfg_;
    movel_laser_filters::FootprintFilterConfig footprintcfg_;
    movel_laser_filters::IntensityFilterConfig intensitycfg_;
    movel_laser_filters::PolygonFilterConfig polygoncfg_;
    movel_laser_filters::RangeFilterConfig rangecfg_;
    movel_laser_filters::ScanShadowsFilterConfig shadowcfg_;
    movel_laser_filters::SpeckleFilterConfig specklecfg_;
};

bool DynLaser::initDynClients(std::string profile_id)
{
    std::string parampath = static_cast<std::string>(node_name_ + profile_id);
    // std::cout << parampath << std::endl;
    // ROS_INFO(parampath.c_str());
    XmlRpc::XmlRpcValue config_xml; 
    if (ros::param::get(parampath, config_xml))
    {
        YAML::Node config = parseStruct(config_xml, parampath);
        for (int i = 0; i < config["scan_filter_chain"].size(); i++) //iterate through all the filters in selected profile
        {
            std::string filter_type = config["scan_filter_chain"][i]["type"].as<std::string>();
            std::string filter_name = config["scan_filter_chain"][i]["name"].as<std::string>();

            if (filter_type == "movel_laser_filters/LaserScanAngularBoundsFilter"){
                dyn_client_angle_.reset(new dynamic_reconfigure::Client<movel_laser_filters::AngularBoundsFilterConfig>(node_name_ + filter_name));
            }
            else if (filter_type == "movel_laser_filters/LaserScanAngularBoundsFilterInPlace"){
                dyn_client_angleinplace_.reset(new dynamic_reconfigure::Client<movel_laser_filters::AngularBoundsFilterInPlaceConfig>(node_name_ + filter_name));
            }
            else if (filter_type == "movel_laser_filters/LaserScanBoxFilter"){
                dyn_client_box_.reset(new dynamic_reconfigure::Client<movel_laser_filters::BoxFilterConfig>(node_name_ + filter_name));
            }
            else if (filter_type == "movel_laser_filters/LaserScanFootprintFilter"){
                dyn_client_footprint_.reset(new dynamic_reconfigure::Client<movel_laser_filters::FootprintFilterConfig>(node_name_ + filter_name));
            }
            else if (filter_type == "movel_laser_filters/LaserScanIntensityFilter"){
                dyn_client_intensity_.reset(new dynamic_reconfigure::Client<movel_laser_filters::IntensityFilterConfig>(node_name_ + filter_name));
            }
            else if (filter_type == "movel_laser_filters/LaserScanPolygonFilter"){
                dyn_client_polygon_.reset(new dynamic_reconfigure::Client<movel_laser_filters::PolygonFilterConfig>(node_name_ + filter_name));
            }
            else if (filter_type == "movel_laser_filters/LaserScanRangeFilter"){
                dyn_client_range_.reset(new dynamic_reconfigure::Client<movel_laser_filters::RangeFilterConfig>(node_name_ + filter_name));
            }
            else if (filter_type == "movel_laser_filters/ScanShadowsFilter"){
                dyn_client_shadow_.reset(new dynamic_reconfigure::Client<movel_laser_filters::ScanShadowsFilterConfig>(node_name_ + filter_name));
            }
            else if (filter_type == "movel_laser_filters/LaserScanSpeckleFilter"){
                dyn_client_speckle_.reset(new dynamic_reconfigure::Client<movel_laser_filters::SpeckleFilterConfig>(node_name_ + filter_name));
            }
        }
        
    }
    else {
        return false;
    }
    return true;
}

bool DynLaser::onProfileUpdate(movel_laser_filters::ProfileUpdate::Request &req, movel_laser_filters::ProfileUpdate::Response &res)
{
    std::string profile_id = req.id;
    XmlRpc::XmlRpcValue xml_array;

    if(ros::param::getCached(node_name_ + profile_id + "/scan_filter_chain/", xml_array)) //type array
    {
        profileUpdateXML(xml_array); 
        res.success = true;
        return true;
    }
    else 
    {
        res.success = false;
        return true;
    }
}

void DynLaser::profileUpdateXML(XmlRpc::XmlRpcValue xml_array)
{
    for(int i=0; i <xml_array.size(); i++)
    {
        std::string filter_type = static_cast<std::string>(xml_array[i]["type"]);

        if(filter_type == "movel_laser_filters/LaserScanAngularBoundsFilter") 
        {
            XMLtoDouble(xml_array[i]["params"]["lower_angle"], anglecfg_.lower_angle );
            XMLtoDouble(xml_array[i]["params"]["upper_angle"], anglecfg_.upper_angle );
            anglecfg_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            dyn_client_angle_->setConfiguration(anglecfg_);
        }
        else if (filter_type == "movel_laser_filters/LaserScanAngularBoundsFilterInPlace")
        {
            XMLtoDouble(xml_array[i]["params"]["lower_angle"], angleinplacecfg_.lower_angle );
            XMLtoDouble(xml_array[i]["params"]["upper_angle"], angleinplacecfg_.upper_angle );
            angleinplacecfg_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            dyn_client_angleinplace_->setConfiguration(angleinplacecfg_);
        }
        else if ( filter_type == "movel_laser_filters/LaserScanBoxFilter")
        {
            XMLtoDouble(xml_array[i]["params"]["max_x"], boxcfg_.max_x);
            XMLtoDouble(xml_array[i]["params"]["max_y"], boxcfg_.max_y);
            XMLtoDouble(xml_array[i]["params"]["max_z"], boxcfg_.max_z);
            XMLtoDouble(xml_array[i]["params"]["min_x"], boxcfg_.min_x);
            XMLtoDouble(xml_array[i]["params"]["min_y"], boxcfg_.min_y);
            XMLtoDouble(xml_array[i]["params"]["min_z"], boxcfg_.min_z);
            boxcfg_.invert = static_cast<bool>(xml_array[i]["params"]["invert"]);
            boxcfg_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            dyn_client_box_->setConfiguration(boxcfg_);
        }
        else if ( filter_type == "movel_laser_filters/LaserScanFootprintFilter")
        {
            XMLtoDouble(xml_array[i]["params"]["inscribed_radius"], footprintcfg_.inscribed_radius);
            footprintcfg_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            dyn_client_footprint_->setConfiguration(footprintcfg_);
        }

        else if ( filter_type == "movel_laser_filters/LaserScanIntensityFilter")
        {
            XMLtoDouble(xml_array[i]["params"]["lower_threshold"], intensitycfg_.lower_threshold);
            XMLtoDouble(xml_array[i]["params"]["upper_threshold"], intensitycfg_.upper_threshold);
            intensitycfg_.invert = static_cast<bool>(xml_array[i]["params"]["invert"]);
            intensitycfg_.filter_override_range = static_cast<bool>(xml_array[i]["params"]["filter_override_range"]);
            intensitycfg_.filter_override_intensity = static_cast<bool>(xml_array[i]["params"]["filter_override_intensity"]);
            intensitycfg_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            dyn_client_intensity_->setConfiguration(intensitycfg_);
        }
        else if ( filter_type == "movel_laser_filters/LaserScanPolygonFilter")
        {
            polygoncfg_.polygon = polygonToString(xml_array[i]["params"]["polygon"]);
            XMLtoDouble(xml_array[i]["params"]["polygon_padding"], polygoncfg_.polygon_padding);
            polygoncfg_.invert = static_cast<bool>(xml_array[i]["params"]["invert"]);
            polygoncfg_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            dyn_client_polygon_->setConfiguration(polygoncfg_);
        }    
        else if ( filter_type == "movel_laser_filters/LaserScanRangeFilter")
        {
            rangecfg_.use_message_range_limits = static_cast<bool>(xml_array[i]["params"]["use_message_range_limits"]);
            XMLtoDouble(xml_array[i]["params"]["lower_threshold"], rangecfg_.lower_threshold);
            XMLtoDouble(xml_array[i]["params"]["upper_threshold"], rangecfg_.upper_threshold);
            XMLtoDouble(xml_array[i]["params"]["lower_replacement_value"], rangecfg_.lower_replacement_value);
            XMLtoDouble(xml_array[i]["params"]["upper_replacement_value"], rangecfg_.upper_replacement_value);
            rangecfg_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            dyn_client_range_->setConfiguration(rangecfg_);
        }
        else if ( filter_type == "movel_laser_filters/ScanShadowsFilter")
        {
            XMLtoDouble(xml_array[i]["params"]["min_angle"], shadowcfg_.min_angle );
            XMLtoDouble(xml_array[i]["params"]["max_angle"], shadowcfg_.max_angle );
            shadowcfg_.window = static_cast<int>(xml_array[i]["params"]["window"]);
            shadowcfg_.neighbors = static_cast<int>(xml_array[i]["params"]["neighbors"]);
            shadowcfg_.remove_shadow_start_point = static_cast<bool>(xml_array[i]["params"]["remove_shadow_start_point"]);
            shadowcfg_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            dyn_client_shadow_->setConfiguration(shadowcfg_);
        }        
        else if ( filter_type  == "movel_laser_filters/LaserScanSpeckleFilter")
        {
            specklecfg_.filter_type = static_cast<int>(xml_array[i]["params"]["filter_type"]);
            XMLtoDouble(xml_array[i]["params"]["max_angle"], specklecfg_.max_range );
            XMLtoDouble(xml_array[i]["params"]["max_range_difference"], specklecfg_.max_range_difference );
            specklecfg_.filter_window = static_cast<int>(xml_array[i]["params"]["filter_window"]);
            specklecfg_.switch_ = static_cast<bool>(xml_array[i]["params"]["switch_"]);
            dyn_client_speckle_->setConfiguration(specklecfg_);
        }       
    }
}

void DynLaser::XMLtoDouble(XmlRpc::XmlRpcValue xml, double &value)
{
    if (xml.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        value = static_cast<double>(xml);
    else
        value = static_cast<int>(xml);
}

std::string DynLaser::polygonToString(const XmlRpc::XmlRpcValue& polygon_xml)
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



/**
 * NOTE: NOT IN USE as it is replaced by profileUpdateXML
 * Parses through selected YAML config and updates the dynamic reconfigure servers based on these
 * parameters. 
 * 
 * @param config YAML config of selected profile to be updated with
 * @see onProfileUpdate
 */
// void DynLaser::profileUpdate(YAML::Node config) 
// {
//     for (int i = 0; i < config["scan_filter_chain"].size(); i++) //iterate through all the filters in a single profile 
//     {
//         if ( config["scan_filter_chain"][i]["type"].as<std::string>() == "movel_laser_filters/LaserScanAngularBoundsFilter")
//         {
//             anglecfg_.lower_angle = config["scan_filter_chain"][i]["params"]["lower_angle"].as<double>();
//             anglecfg_.upper_angle = config["scan_filter_chain"][i]["params"]["upper_angle"].as<double>();
//             anglecfg_.switch_ = config["scan_filter_chain"][i]["params"]["switch_"].as<bool>();
//             dyn_client_angle_->setConfiguration(anglecfg_);
//         }
//         else if ( config["scan_filter_chain"][i]["type"].as<std::string>() == "movel_laser_filters/LaserScanAngularBoundsFilterInPlace")
//         {
//             angleinplacecfg_.lower_angle = config["scan_filter_chain"][i]["params"]["lower_angle"].as<double>();
//             angleinplacecfg_.upper_angle = config["scan_filter_chain"][i]["params"]["upper_angle"].as<double>();
//             angleinplacecfg_.switch_ = config["scan_filter_chain"][i]["params"]["switch_"].as<bool>();
//             dyn_client_angleinplace_->setConfiguration(angleinplacecfg_);
//         }
//         else if ( config["scan_filter_chain"][i]["type"].as<std::string>() == "movel_laser_filters/LaserScanBoxFilter")
//         {
//             boxcfg_.max_x = config["scan_filter_chain"][i]["params"]["max_x"].as<double>();
//             boxcfg_.max_y = config["scan_filter_chain"][i]["params"]["max_y"].as<double>();
//             boxcfg_.max_z = config["scan_filter_chain"][i]["params"]["max_z"].as<double>();
//             boxcfg_.min_x = config["scan_filter_chain"][i]["params"]["min_x"].as<double>();
//             boxcfg_.min_y = config["scan_filter_chain"][i]["params"]["min_y"].as<double>();
//             boxcfg_.min_z = config["scan_filter_chain"][i]["params"]["min_z"].as<double>();
//             boxcfg_.invert = config["scan_filter_chain"][i]["params"]["invert"].as<bool>(); 
//             boxcfg_.switch_ = config["scan_filter_chain"][i]["params"]["switch_"].as<bool>();
//             dyn_client_box_->setConfiguration(boxcfg_);
//         }
//         else if ( config["scan_filter_chain"][i]["type"].as<std::string>() == "movel_laser_filters/LaserScanFootprintFilter")
//         {
//             footprintcfg_.inscribed_radius = config["scan_filter_chain"][i]["params"]["inscribed_radius"].as<double>();
//             footprintcfg_.switch_ = config["scan_filter_chain"][i]["params"]["switch_"].as<bool>();
//             dyn_client_footprint_->setConfiguration(footprintcfg_);
//         }
//         else if ( config["scan_filter_chain"][i]["type"].as<std::string>() == "movel_laser_filters/LaserScanIntensityFilter")
//         {
//             intensitycfg_.lower_threshold = config["scan_filter_chain"][i]["params"]["lower_threshold"].as<double>();
//             intensitycfg_.upper_threshold = config["scan_filter_chain"][i]["params"]["upper_threshold"].as<double>();
//             intensitycfg_.invert = config["scan_filter_chain"][i]["params"]["invert"].as<bool>();
//             intensitycfg_.filter_override_range = config["scan_filter_chain"][i]["params"]["filter_override_range"].as<bool>();
//             intensitycfg_.filter_override_intensity = config["scan_filter_chain"][i]["params"]["filter_override_intensity"].as<bool>();
//             intensitycfg_.switch_ = config["scan_filter_chain"][i]["params"]["switch_"].as<bool>();
//             dyn_client_intensity_->setConfiguration(intensitycfg_);
//         }
//         else if ( config["scan_filter_chain"][i]["type"].as<std::string>() == "movel_laser_filters/LaserScanPolygonFilter")
//         {
//             // polygoncfg_.polygon = config["scan_filter_chain"][i]["params"]["polygon"].as<std::string>();
//             polygoncfg_.polygon_padding = config["scan_filter_chain"][i]["params"]["polygon_padding"].as<double>();
//             polygoncfg_.invert = config["scan_filter_chain"][i]["params"]["invert"].as<bool>();
//             polygoncfg_.switch_ = config["scan_filter_chain"][i]["params"]["switch_"].as<bool>();
//             dyn_client_polygon_->setConfiguration(polygoncfg_);
//         }
//         else if ( config["scan_filter_chain"][i]["type"].as<std::string>() == "movel_laser_filters/LaserScanRangeFilter")
//         {
//             rangecfg_.use_message_range_limits = config["scan_filter_chain"][i]["params"]["use_message_range_limits"].as<bool>();
//             rangecfg_.lower_threshold = config["scan_filter_chain"][i]["params"]["lower_threshold"].as<double>();
//             rangecfg_.upper_threshold = config["scan_filter_chain"][i]["params"]["upper_threshold"].as<double>();
//             rangecfg_.lower_replacement_value = config["scan_filter_chain"][i]["params"]["lower_replacement_value"].as<double>();
//             rangecfg_.upper_replacement_value = config["scan_filter_chain"][i]["params"]["upper_replacement_value"].as<double>();
//             rangecfg_.switch_ = config["scan_filter_chain"][i]["params"]["switch_"].as<bool>();
//             dyn_client_range_->setConfiguration(rangecfg_);
//         }
//         else if ( config["scan_filter_chain"][i]["type"].as<std::string>() == "movel_laser_filters/ScanShadowsFilter")
//         {
//             shadowcfg_.min_angle = config["scan_filter_chain"][i]["params"]["min_angle"].as<double>();
//             shadowcfg_.max_angle = config["scan_filter_chain"][i]["params"]["max_angle"].as<double>();
//             shadowcfg_.window = config["scan_filter_chain"][i]["params"]["window"].as<int>();
//             shadowcfg_.neighbors = config["scan_filter_chain"][i]["params"]["neighbors"].as<int>();
//             shadowcfg_.remove_shadow_start_point = config["scan_filter_chain"][i]["params"]["remove_shadow_start_point"].as<bool>();
//             shadowcfg_.switch_ = config["scan_filter_chain"][i]["params"]["switch_"].as<bool>();
//             dyn_client_shadow_->setConfiguration(shadowcfg_);
//         }        
//         else if ( config["scan_filter_chain"][i]["type"].as<std::string>() == "movel_laser_filters/LaserScanSpeckleFilter")
//         {
//             specklecfg_.filter_type = config["scan_filter_chain"][i]["params"]["filter_type"].as<int>();
//             specklecfg_.max_range = config["scan_filter_chain"][i]["params"]["max_range"].as<double>();
//             specklecfg_.max_range_difference = config["scan_filter_chain"][i]["params"]["max_range_difference"].as<double>();
//             specklecfg_.filter_window = config["scan_filter_chain"][i]["params"]["filter_window"].as<int>();
//             specklecfg_.switch_ = config["scan_filter_chain"][i]["params"]["switch_"].as<bool>();
//             dyn_client_speckle_->setConfiguration(specklecfg_);
//         }        
//     }
// }
