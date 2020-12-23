#ifndef CC_STATE_MAP_H
#define CC_STATE_MAP_H
#include <g3log/g3log.hpp>

// Events
struct EvToMap
{
  //int i = 2;
};

// Guards
struct GuardEntryMap
{
  bool operator()() const { LOG(INFO) << "guard entry map"; return true; }
};
struct GuardToMap
{
  bool operator()() const { LOG(INFO) << "guard to map"; return true; }
};
struct GuardExitMap
{
  bool operator()() const { LOG(INFO) << "guard exit map"; return true; }
};
struct GuardUnexpectedMap
{
  bool operator()() const { LOG(INFO) << "guard unexpected map"; return true; }
};
struct GuardExceptionMap
{
  bool operator()() const { LOG(INFO) << "guard unexpected map"; return true; }
};

// Actions
struct ActEntryMap
{
  void operator()() { LOG(INFO) << "action entry map"; }
};
struct ActToMap
{
  void operator()() { LOG(INFO) << "action to map"; }
};
struct ActExitMap
{
  void operator()() { LOG(INFO) << "action exit map"; }
};
struct ActUnexpectedMap
{
  void operator()() { LOG(INFO) << "action unexpected map"; }
};
struct ActExceptionMap
{
  void operator()() { LOG(INFO) << "action unexpected map"; }
};


#endif

