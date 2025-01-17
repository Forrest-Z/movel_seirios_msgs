#include <map_editor/map_editor.h>
#include <movel_hasp_vendor/license.h>

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "map_editor");
  ros::NodeHandle nh("~");
  MapEditor poly_map_update;
  poly_map_update.loadParams();
  poly_map_update.make_multi_polygons_ = nh.advertiseService("multi_updates", &MapEditor::updateMultiPolygonsCb, &poly_map_update);
  poly_map_update.restore_map_ = nh.advertiseService("restore", &MapEditor::restoreMapCb, &poly_map_update);
  poly_map_update.relaunch_map_server_ = nh.serviceClient<std_srvs::Trigger>(poly_map_update.relaunch_map_service_);
  ros::spin();

#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return 0;
}
