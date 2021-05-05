#include <map_editor/map_editor.h>

bool MapEditor::updateCb(movel_seirios_msgs::DrawPolygon::Request &req,
                         movel_seirios_msgs::DrawPolygon::Response &res)
{
    filename_ = req.filename;
    try
    {
        std::vector<cv::Point> poly_points_;
        std::string nav_path = nav_map_path_ + filename_ + ".pgm";
        cv::Mat image = cv::imread(nav_path, cv::IMREAD_GRAYSCALE );
        for (movel_seirios_msgs::Pixel& pixel : req.pixels)
        {
            cv::Point pt;
            pt.x = pixel.x;
            pt.y = pixel.y;
            poly_points_.push_back(pt);
        }

        if (poly_points_.size() > 2)
        {
            cv::polylines(image, poly_points_, true, 0, line_width_, 8);
            std::vector<std::vector<cv::Point>> contours;
            contours.push_back(poly_points_);
            cv::fillPoly(image, contours, 0, 8);
        }
        else if (poly_points_.size() == 2)
        {
            cv::line(image, poly_points_[0], poly_points_[1], 0, line_width_, 8);
        }
        else if (poly_points_.size() < 2)
        {
            ROS_INFO("Cannot process the number of points below 2!");
            res.success = false;
            return false;
        }
        cv::Mat image_gray = image;
        cv::imwrite(nav_path, image_gray);

        std_srvs::Trigger relaunch;
        if(!relaunch_map_server_.call(relaunch))
            ROS_INFO("Failed to call relaunch service for localization handler");
    }
    catch (cv::Exception& e)
    {   
        const char* err_msg = e.what();
        std::cerr << err_msg << std::endl;
        ROS_INFO("[Map Editor] %s", err_msg);
        res.success = false;
        return false;
    }
    res.success = true;
    return true;
}

bool MapEditor::restoreMapCb(movel_seirios_msgs::DrawPolygon::Request &req,
                             movel_seirios_msgs::DrawPolygon::Response &res)
{
    filename_ = req.filename;
    std::string loc_path = loc_map_path_ + filename_ + ".pgm";
    std::string nav_path = nav_map_path_ + filename_ + ".pgm";
    cv::Mat image = cv::imread(loc_path, cv::IMREAD_GRAYSCALE );
    cv::Mat image_gray = image;
    cv::imwrite(nav_path, image_gray);
    std_srvs::Trigger relaunch;
    if(!relaunch_map_server_.call(relaunch))
        ROS_INFO("Failed to call relaunch service for localization handler");
    res.success = true;
    return true;
}                                


void MapEditor::loadParams()
{
    ros::NodeHandle nh_("~");
    nh_.getParam("navigation_map_path", nav_map_path_);
    nh_.getParam("localization_map_path", loc_map_path_);
    nh_.getParam("line_width", line_width_);
}