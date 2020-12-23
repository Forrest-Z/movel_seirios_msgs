#ifndef CC_STATE_IDLE_H
#define CC_sTATE_IDLE_H
#include <g3log/g3log.hpp>

// Events
struct EvToIdle {};

// Guards
struct GuardEntryIdle
{
  bool operator()() const { LOG(INFO) << "guard entry idle"; return true; }
};
struct GuardToIdle
{
  bool operator()() const { LOG(INFO) << "guard to idle"; return true; }
};
struct GuardExitIdle
{
  bool operator()() const { LOG(INFO) << "guard exit idle"; return true; }
};
struct GuardUnexpectedIdle
{
  bool operator()() const { LOG(INFO) << "guard unexpected idle"; return true; }
};
struct GuardExceptionIdle
{
  bool operator()() const { LOG(INFO) << "guard unexpected idle"; return true; }
};


// Actions
struct ActEntryIdle
{
  void operator()() { LOG(INFO) << "action entry idle"; }
};
struct ActToIdle
{
  void operator()() { LOG(INFO) << "action to idle"; }
};
struct ActExitIdle
{
  void operator()() { LOG(INFO) << "action exit idle"; }
};
struct ActUnexpectedIdle
{
  void operator()() { LOG(INFO) << "action unexpected idle"; }
};
struct ActExceptionIdle
{
  void operator()() { LOG(INFO) << "action unexpected idle"; }
};
#endif
