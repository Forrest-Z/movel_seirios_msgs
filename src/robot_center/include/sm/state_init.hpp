#ifndef ROBOT_STATE_INIT_H
#define ROBOT_STATE_INIT_H
#include <movel_g3log_vendor/include/g3log/g3log.hpp>
#include <uv.h>
//#include <helpers/rabbit_helper.hpp>
//#include <helpers/mongo_helper.hpp>
//#include <helpers/ros_helper.hpp>

struct EvToInit
{
};

// Guards
struct GuardEntryInit  // guardEntryInit;
{
  bool operator()() const
  {
    LOG(INFO) << "guard exit init";
    return true;
  }
};
struct GuardToInit  // guardToInit;
{
  bool operator()() const
  {
    LOG(INFO) << "guard exit init";
    return true;
  }
};
struct GuardExitInit
{
  bool operator()() const
  {
    LOG(INFO) << "guard exit init";
    return true;
  }
};
struct GuardUnexpectedInit
{
  bool operator()() const
  {
    LOG(INFO) << "guard unexpected init";
    return true;
  }
};
struct GuardExceptionInit
{
  bool operator()() const
  {
    LOG(INFO) << "guard unexpected init";
    return true;
  }
};

// Actions
struct ActEntryInit  // {} actEntryInit;
{
  void operator()()
  {
    LOG(INFO) << "action exit init";
  }
};
struct ActToInit  // {} actToInit;
{
  void operator()()
  {
    LOG(INFO) << "action exit init";
  }
};
struct ActExitInit
{
  void operator()()
  {
    LOG(INFO) << "action exit init";
  }
};
struct ActUnexpectedInit
{
  void operator()()
  {
    LOG(INFO) << "action unexpected init";
  }
};
struct ActExceptionInit
{
  void operator()()
  {
    LOG(INFO) << "action unexpected init";
  }
};

#endif
