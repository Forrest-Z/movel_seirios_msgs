#include <sm/state_init.hpp>

/*

struct guardEntryInit
{
  bool operator()() const {
    try
    {
      LOG(INFO) << "*** Guard for Entry to Init ***";
      return true;
    }
    catch (...)
    {
      std::cout << "Logger not initialized" << std::endl;
      return false;
    }
  }
} guardEntryInit;

struct ActEntryInit
{
  void operator()()
  {
    LOG(INFO) << "*** Action for Entry to Init ***";
    //uv_loop_t *loop = malloc(sizeof(uv_loop_t));
    //int loopi_init = uv_loop_init(loop);
    LOG(INFO) << "Initializing Rabbit_Helper";
    auto *loop = uv_default_loop();
    RabbitHelper rabbit(loop);
    LOG(INFO) << "Initializing Mongo_Helper";
    LOG(INFO) << "Initializing ROS_Helper";
  }

} actEntryInit;

struct guardToInit
{
  bool operator()() const
  {
    LOG(INFO) << "*** Guard for Init ***";
    LOG(INFO) << "Checking existance of ROS, MongoDB and RabbitMQ";
 //   if (RosHelper::rosExists() and MongoHelper::mongoExists() and RabbitHelper::rabbitExists())
 //   {
 //     LOG(INFO) << "ROS, MongoDB and RabbitMQ running";
 //     return true;
 //   {
 //   else
 //   {
 //     LOG(FATAL) << "ROS, MongoDB and RabbitMQ not";
 //     return false;
 //   }
  }
} guardToInit;

struct ActToInit
{
  void operator()() { LOG(INFO) << "action to init"; }
} actToInit;
*/
