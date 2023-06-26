#include <movel_zone_coverage/zone_coverage_server.h>

#include <movel_hasp_vendor/license.h>
#include <ros/xmlrpc_manager.h>

#include <signal.h>

sig_atomic_t volatile g_request_shutdown = 0;

void onXmlRpcShutdown(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("[zone_coverage_server] Shutdown request received. Reason: %s", reason.c_str());
    // Set flag
    g_request_shutdown = 1;
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "zone_coverage_server", ros::init_options::NoSigintHandler);

  // capture SIGINT signal
  signal(SIGINT, [](int sig) {
    ROS_WARN("[zone_coverage_server] Received SIGINT signal, shutting down");
    // set flag
    g_request_shutdown = 1;
  });

  // override XMLRPC shutdown request handler
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", onXmlRpcShutdown);

  ros::NodeHandle nh;
  ZoneCoverageServer zcs(nh);

  ros::Rate r(20.0);
  while (!g_request_shutdown)
  {
    ros::spinOnce();
    r.sleep();
  }

  zcs.shutdownHandler();

#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return (0);
}