#include <map_editor/map_editor.h>
#include <movel_hasp_vendor/license.h>

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml(19);
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "map_editor");
  ros::NodeHandle nh("~");
  MapEditor poly_map_update;
  poly_map_update.loadParams();
  poly_map_update.make_polygon_ = nh.advertiseService("update", &MapEditor::updateCb, &poly_map_update);
  poly_map_update.restore_map_ = nh.advertiseService("restore", &MapEditor::restoreMapCb, &poly_map_update);
  poly_map_update.relaunch_map_server_ = nh.serviceClient<std_srvs::Trigger>("/task_supervisor/localization_handler/"
                                                                             "relaunch_map");
  ros::spin();

#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return 0;
}
