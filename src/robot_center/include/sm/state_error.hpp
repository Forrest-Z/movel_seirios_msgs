#ifndef CC_STATE_ERROR_H
#define CC_STATE_ERROR_H
#include <g3log/g3log.hpp>

// Events
struct EvToError
{
  //int i = 2;
};

// Guards
struct GuardEntryError
{
  bool operator()() const { LOG(INFO) << "guard entry entry"; return true; }
};
struct GuardToError
{
  bool operator()() const { LOG(INFO) << "guard to error"; return true; }
};
struct GuardExitError
{
  bool operator()() const { LOG(INFO) << "guard exit error"; return true; }
};
struct GuardUnexpectedError
{
  bool operator()() const { LOG(INFO) << "guard unexpected error"; return true; }
};
struct GuardExceptionError
{
  bool operator()() const { LOG(INFO) << "guard unexpected error"; return true; }
};

// Actions
struct ActEntryError
{
  void operator()() { LOG(INFO) << "action entry error"; }
};
struct ActToError
{
  void operator()() { LOG(INFO) << "action to error"; }
};
struct ActExitError
{
  void operator()() { LOG(INFO) << "action exit error"; }
};
struct ActUnexpectedError
{
  void operator()() { LOG(INFO) << "action unexpected error"; }
};
struct ActExceptionError
{
  void operator()() { LOG(INFO) << "action unexpected error"; }
};
#endif
