#ifndef CC_STATE_PREP_H
#define CC_STATE_PREP_H
#include <g3log/g3log.hpp>
namespace sml = boost::sml;

// Events
struct EvToPrep {};

// Guards
struct GuardEntryPrep
{
  bool operator()() const { LOG(INFO) << "guard entry prep"; return true; }
};
struct GuardToPrep
{
  bool operator()() const { LOG(INFO) << "guard to prep"; return true; }
};
struct GuardExitPrep
{
  bool operator()() const { LOG(INFO) << "guard exit prep"; return true; }
};
struct GuardUnexpectedPrep
{
  bool operator()() const { LOG(INFO) << "guard unexpected prep"; return true; }
};
struct GuardExceptionPrep
{
  bool operator()() const { LOG(INFO) << "guard unexpected prep"; return true; }
};

// Actions
struct ActEntryPrep
{
  void operator()() { LOG(INFO) << "action entry prep"; }
};
struct ActToPrep
{
  void operator()() { LOG(INFO) << "action to prep"; }
};
struct ActExitPrep
{
  void operator()() { LOG(INFO) << "action exit prep"; }
};
struct ActUnexpectedPrep
{
  void operator()() { LOG(INFO) << "action unexpected prep"; }
};
struct ActExceptionPrep
{
  void operator()() { LOG(INFO) << "action unexpected prep"; }
};

#endif
