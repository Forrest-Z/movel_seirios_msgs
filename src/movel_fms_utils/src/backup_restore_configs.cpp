#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <movel_seirios_msgs/BackupConfig.h>
#include <movel_seirios_msgs/RestoreConfig.h>
#include <movel_seirios_msgs/StringTrigger.h>

namespace fs = boost::filesystem;

class ConfigManager
{
public:
    ConfigManager(const std::string& config_path, const std::string& backup_path)
        : config_path_(config_path), backup_path_(backup_path)
    {
        ROS_INFO_STREAM("Config Manager ready.");
    }

    bool backupConfig(movel_seirios_msgs::BackupConfig::Request &req,
                      movel_seirios_msgs::BackupConfig::Response &res)
    {
        std::string timestamp = getCurrentTimestamp();
        std::string config_folder_name = config_path_.filename().string();
        std::string backup_folder_name = config_folder_name + "_backup_" + req.software_version + "_" + timestamp;
        fs::path backup_dir = backup_path_ / backup_folder_name;

        ROS_INFO_STREAM("Creating backup directory: " << backup_dir);

        try
        {
            // Use copyDir function to copy the directory contents recursively.
            copyDir(config_path_, backup_dir);

            res.backup_id = backup_folder_name;
            res.success = true;
            res.message = "Backup successful.";
            return true;
        }
        catch (const fs::filesystem_error& e)
        {
            res.success = false;
            res.message = "Error during backup: " + std::string(e.what());
            return false;
        }
    }

    void copyDir(const fs::path& source, const fs::path& destination)
    {
        if (!fs::exists(source) || !fs::is_directory(source))
            throw std::runtime_error("Source directory " + source.string() + " does not exist or is not a directory.");

        if (!fs::exists(destination))
        {
            if (!fs::create_directory(destination))
                throw std::runtime_error("Cannot create destination directory " + destination.string() + ".");
        }
        for (const auto& entry : fs::directory_iterator(source))
        {
            const fs::path& current = entry.path();
            // Skip if the file/folder name starts with a dot (.)
            if (current.filename().string().find('.') == 0) {
                continue;
            }
            if (fs::is_directory(current))
            {
                // Recursively copy subdirectories.
                copyDir(current, destination / current.filename());
            }
            else
            {
                // Copy regular file with the option to overwrite existing files.
                fs::copy_file(current, destination / current.filename(), fs::copy_option::overwrite_if_exists);
            }
        }
    }

    bool getSoftwareVersion(movel_seirios_msgs::StringTrigger::Request &req,
                            movel_seirios_msgs::StringTrigger::Response &res)
    {
        std::string version;
        try
        {
            std::ifstream file((config_path_ / "seirios_config_release.yml").string());
            std::string line;

            if (file.is_open()) {
                while (getline(file, line)) {
                    std::string key = "Version:";
                    size_t pos = line.find(key);
                    if (pos != std::string::npos) {
                        // Skip the key itself to start at the version number
                        // pos + key.length() will give you the start position of the version number
                        version = line.substr(pos + key.length());
                        if (!version.empty() && version[0] == ' ') {
                            // If there is a space directly after the colon, remove it
                            version.erase(0, 1);
                        }
                        break; // Assuming there's only one version line in the file
                    }
                }
                file.close();
        
                if (!version.empty()) {
                    std::cout << "Version: " << version << std::endl;
                } else {
                    std::cerr << "Version not found." << std::endl;
                }
            }
            else {
                std::cerr << "Unable to open file." << std::endl;
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            res.success = false;
            res.message = "Error during get software version: " + std::string(e.what());
            return false;
        }
        
        res.success = true;
        res.message = version;
        return true;
    }

    bool restoreConfig(movel_seirios_msgs::RestoreConfig::Request &req,
                       movel_seirios_msgs::RestoreConfig::Response &res)
    {
        // backup first before restoring
        try
        {
            std::string config_folder_name = config_path_.filename().string();
            std::string backup_before_restore_folder_name = config_folder_name + "_backup_before_restore";
            copyDir(config_path_, backup_path_ / backup_before_restore_folder_name);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            res.success = false;
            res.message = "Error during restoration: " + std::string(e.what());
            return false;
        }
        catch(const std::runtime_error& e)
        {
            std::cerr << e.what() << '\n';
            res.success = false;
            res.message = "Error during restoration: " + std::string(e.what());
            return false;
        }

        fs::path backup_dir = backup_path_ / req.backup_id;
        if (!fs::exists(backup_dir) || !fs::is_directory(backup_dir))
        {
            res.success = false;
            res.message = "Backup directory does not exist.";
            return false;
        }

        try
        {
            // Remove current config directory and copy from backup
            // fs::remove_all(config_path_);
            for (const auto & entry : fs::directory_iterator(config_path_)) {fs::remove_all(entry.path());}
            
            copyDir(backup_dir, config_path_);

            res.success = true;
            res.message = "Restore successful.";
            return true;
        }
        catch (const fs::filesystem_error& e)
        {
            //copy back again from backup_before_restore
            std::string config_folder_name = config_path_.filename().string();
            std::string backup_before_restore_folder_name = config_folder_name + "_backup_before_restore";
            copyDir(backup_path_ / backup_before_restore_folder_name, config_path_);

            res.success = false;
            res.message = "Error during restoration: " + std::string(e.what());
            return false;
        }
    }

private:
    std::string getCurrentTimestamp()
    {
        const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
        const boost::posix_time::time_duration td = now.time_of_day();
        const long hours = td.hours();
        const long minutes = td.minutes();
        const long seconds = td.seconds();
        const boost::gregorian::date date = now.date();
        const boost::gregorian::date::ymd_type ymd = date.year_month_day();
        const long day = ymd.day;
        const boost::gregorian::greg_month month = ymd.month;
        const long year = ymd.year;

        std::stringstream ss;
        ss << year << "-" << month.as_short_string() << "-" << day << "_"
           << hours << "-" << minutes << "-" << seconds;
        return ss.str();
    }

    fs::path config_path_;
    fs::path backup_path_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "config_manager_node");

    std::string config_path = "/home/movel/.config/movel/config";
    std::string backup_path = "/home/movel/.config/movel/backup/";
    ConfigManager manager(config_path, backup_path);
    
    ros::NodeHandle nh;
    
    ros::ServiceServer backup_service = nh.advertiseService("/backup_config", &ConfigManager::backupConfig, &manager);
    ros::ServiceServer restore_service = nh.advertiseService("/restore_config", &ConfigManager::restoreConfig, &manager);
    ros::ServiceServer get_software_version = nh.advertiseService("/get_software_version", &ConfigManager::getSoftwareVersion, &manager);
        
    ros::spin();

    return 0;
}
