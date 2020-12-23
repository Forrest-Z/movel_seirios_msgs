#ifndef CC_STATE_MISSION_H
#define CC_STATE_MISSION_H
#include <g3log/g3log.hpp>

// Events
struct EvToMission
{
  //int i = 3;
};

// Guards
struct GuardEntryMission
{
  bool operator()() const { LOG(INFO) << "guard entry mission"; return true; }
};
struct GuardToMission
{
  bool operator()() const { LOG(INFO) << "guard to mission"; return true; }
};
struct GuardExitMission
{
  bool operator()() const { LOG(INFO) << "guard exit mission"; return true; }
};
struct GuardUnexpectedMission
{
  bool operator()() const { LOG(INFO) << "guard unexpected mission"; return true; }
};
struct GuardExceptionMission
{
  bool operator()() const { LOG(INFO) << "guard unexpected mission"; return true; }
};

// Actions
struct ActEntryMission
{
  void operator()() { LOG(INFO) << "action entry mission"; }
};
struct ActToMission
{
  void operator()() { LOG(INFO) << "action to mission"; }
};
struct ActExitMission
{
  void operator()() { LOG(INFO) << "action exit mission"; }
};
struct ActUnexpectedMission
{
  void operator()() { LOG(INFO) << "action unexpected mission"; }
};
struct ActExceptionMission
{
  void operator()() { LOG(INFO) << "action unexpected mission"; }
};
#endif
