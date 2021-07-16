#include <map_editor/map_editor.h>

bool MapEditor::updateMultiPolygonsCb(movel_seirios_msgs::DrawMultiPolygons::Request &req,
                                     movel_seirios_msgs::DrawMultiPolygons::Response &res)
{
    filename_ = req.filename;
    try
    {
        std::string nav_path = nav_map_path_ + filename_ + ".pgm";
        cv::Mat image = cv::imread(nav_path, cv::IMREAD_GRAYSCALE );
        ROS_INFO("[map editor] Creating %ld polygons...", req.polygons.size());
        for (movel_seirios_msgs::Polygon& polygon : req.polygons)
        {
            std::vector<cv::Point> poly_points_;
            for (movel_seirios_msgs::Pixel& pixel : polygon.pixels)
            {
                cv::Point pt;
                pt.x = pixel.x;
                pt.y = pixel.y;
                poly_points_.push_back(pt);
            }

            if (poly_points_.size() > 2)
            {
                if(!polygon.is_eraser)
                {
                    cv::polylines(image, poly_points_, true, cv::Scalar(0,0,0), 1, 8);
                    std::vector<std::vector<cv::Point>> contours;
                    contours.push_back(poly_points_);
                    cv::fillPoly(image, contours, cv::Scalar(0,0,0), 8);
                    ROS_INFO("[map editor] No-go zone with %ld points has been created!", poly_points_.size());
                }
                else
                {
                    cv::polylines(image, poly_points_, true, cv::Scalar(255,255,255), 1, 8);
                    std::vector<std::vector<cv::Point>> contours;
                    contours.push_back(poly_points_);
                    cv::fillPoly(image, contours, cv::Scalar(255,255,255), 8);
                    ROS_INFO("[map editor] White zone with %ld points has been created!", poly_points_.size());
                }
            }
            else if (poly_points_.size() == 2)
            {
                if(!polygon.is_eraser)
                {
                    cv::line(image, poly_points_[0], poly_points_[1], cv::Scalar(0,0,0), polygon.line_width, 8);
                    ROS_INFO("[map editor] Wall has been created!");
                }
                else
                {
                    cv::line(image, poly_points_[0], poly_points_[1], cv::Scalar(255,255,255), polygon.line_width, 8);
                    ROS_INFO("[map editor] White line has been created!");
                }
            }
            else if (poly_points_.size() < 2)
            {
                ROS_INFO("[map editor] Cannot process the number of points below 2!");
                res.success = false;
                return false;
            }
            cv::Mat image_gray = image;
            cv::imwrite(nav_path, image_gray);
        }

        std_srvs::Trigger relaunch;
        if(!relaunch_map_server_.call(relaunch))
            ROS_INFO("[map editor] Failed to call relaunch service for localization handler");
            
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

bool MapEditor::restoreMapCb(movel_seirios_msgs::DrawMultiPolygons::Request &req,
                             movel_seirios_msgs::DrawMultiPolygons::Response &res)
{
    ROS_INFO("[map_editor] Restoring to the original map!");
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
}