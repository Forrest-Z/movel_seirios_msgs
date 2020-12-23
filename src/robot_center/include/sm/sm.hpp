#ifndef CC_SM_H
#define CC_SM_H

#include <g3log/g3log.hpp>
#include <boost/sml.hpp>

#include <sm/state_init.hpp>
#include <sm/state_idle.hpp>
#include <sm/state_error.hpp>
#include <sm/state_map.hpp>
#include <sm/state_mission.hpp>
#include <sm/state_prep.hpp>


struct EvToShutdown
{
  //int i = 10;
};

// Guards


struct GuardToShutdown
{
  bool operator()() const { LOG(INFO) << "Guard to shutdown"; return true; }
};

// Actions
struct ActToShutdown
{
  void operator()() { LOG(INFO) << "Action to shutdown"; }
};


// Logging
struct Logger
{
    template <class SM, class TEvent>
    void log_process_event(const TEvent&) {
        printf("[%s][process_event] %s\n", boost::sml::aux::get_type_name<SM>(), boost::sml::aux::get_type_name<TEvent>());
    }

    template <class SM, class TGuard, class TEvent>
    void log_guard(const TGuard&, const TEvent&, bool result) {
        printf("[%s][guard] %s %s %s\n", boost::sml::aux::get_type_name<SM>(), boost::sml::aux::get_type_name<TGuard>(), boost::sml::aux::get_type_name<TEvent>(),
               (result ? "[OK]" : "[Reject]"));
    }

    template <class SM, class TAction, class TEvent>
    void log_action(const TAction&, const TEvent&) {
        printf("[%s][action] %s %s\n", boost::sml::aux::get_type_name<SM>(), boost::sml::aux::get_type_name<TAction>(), boost::sml::aux::get_type_name<TEvent>());
    }

    template <class SM, class TSrcState, class TDstState>
    void log_state_change(const TSrcState& src, const TDstState& dst) {
        printf("[%s][transition] %s -> %s\n", boost::sml::aux::get_type_name<SM>(), src.c_str(), dst.c_str());
    }
};

class RobotSM
{
  public:
    class Init;
    class Idle;
    class Error;
    class Map;
    class Mission;
    class Prep;

    auto operator()()
    {
      using namespace sml;
      return make_transition_table(
        // Entry Guards and Actions declaration
        *sml::state<Init> + sml::on_entry<_> [ GuardEntryInit{} ] / ActEntryInit{},
        sml::state<Idle> + sml::on_entry<_> [ GuardEntryIdle{} ] / ActEntryIdle{},
        sml::state<Error> + sml::on_entry<_> [ GuardEntryError{} ] / ActEntryError{},
        sml::state<Map> + sml::on_entry<_> [ GuardEntryMap{} ] / ActEntryMap{},
        sml::state<Mission> + sml::on_entry<_> [ GuardEntryMission{} ] / ActEntryMission{},
        sml::state<Prep> + sml::on_entry<_> [ GuardEntryPrep{} ] / ActEntryPrep{},

        // Exit Guards and Actions declaration
        sml::state<Init> + sml::on_exit<_> [ GuardExitInit{} ] / ActExitInit{},
        sml::state<Idle> + sml::on_exit<_> [ GuardExitIdle{} ] / ActExitIdle{},
        sml::state<Error> + sml::on_exit<_> [ GuardExitError{} ] / ActExitError{},
        sml::state<Map> + sml::on_exit<_> [ GuardExitMap{} ] / ActExitMap{},
        sml::state<Mission> + sml::on_exit<_> [ GuardExitMission{} ] / ActExitMission{},
        sml::state<Prep> + sml::on_exit<_> [ GuardExitPrep{} ] / ActExitPrep{},

        // Event triggers{}, Guards and Actions declaration
        sml::state<Init> + sml::event<EvToIdle> [ GuardToIdle{} ] / ActToIdle{} = sml::state<Idle>,
        sml::state<Init> + sml::event<EvToError> [ GuardToError{} ] / ActToError{} = sml::state<Error>,
        sml::state<Idle> + sml::event<EvToInit> [ GuardToInit{} ] / ActToInit{} = sml::state<Init>,
        sml::state<Idle> + sml::event<EvToError> [ GuardToError{} ] / ActToError{} = sml::state<Error>,
        sml::state<Idle> + sml::event<EvToError> [ GuardToMap{} ] / ActToMap{} = sml::state<Map>,
        sml::state<Idle> + sml::event<EvToError> [ GuardToMission{} ] / ActToMission{} = sml::state<Mission>,
        sml::state<Idle> + sml::event<EvToError> [ GuardToPrep{} ] / ActToPrep{} = sml::state<Prep>,
        sml::state<Error> + sml::event<EvToError> [ GuardToInit{} ] / ActToInit{} = sml::state<Init>,
        sml::state<Error> + sml::event<EvToError> [ GuardToIdle{} ] / ActToIdle{} = sml::state<Idle>,
        sml::state<Map> + sml::event<EvToError> [ GuardToError{} ] / ActToError{} = sml::state<Error>,
        sml::state<Mission> + sml::event<EvToError> [ GuardToError{} ] / ActToError{} = sml::state<Error>,
        sml::state<Prep> + sml::event<EvToError> [ GuardToError{} ] / ActToError{} = sml::state<Error>,

        // Unexpected transition Guards and Action declaration{}, reroutes to Idle sml::state
        sml::state<Init> + sml::unexpected_event<_> [ GuardUnexpectedInit{} ] / ActUnexpectedInit{} = sml::state<Idle>,
        sml::state<Idle> + sml::unexpected_event<_> [ GuardUnexpectedIdle{} ] / ActUnexpectedIdle{} = sml::state<Idle>,
        sml::state<Error> + sml::unexpected_event<_> [ GuardUnexpectedError{} ] / ActUnexpectedError{} = sml::state<Idle>,
        sml::state<Map> + sml::unexpected_event<_> [ GuardUnexpectedMap{} ] / ActUnexpectedMap{} = sml::state<Idle>,
        sml::state<Mission> + sml::unexpected_event<_> [ GuardUnexpectedMission{} ] / ActUnexpectedMission{} = sml::state<Idle>,
        sml::state<Prep> + sml::unexpected_event<_> [ GuardUnexpectedPrep{} ] / ActUnexpectedPrep{} = sml::state<Idle>,

        // Exception transition Guards and Action declaration{}, reroutes to Error sml::state
        sml::state<Init> + sml::exception<_> [ GuardExceptionInit{} ] / ActExceptionInit{} = sml::state<Error>,
        sml::state<Idle> + sml::exception<_> [ GuardExceptionIdle{} ] / ActExceptionIdle{} = sml::state<Error>,
        sml::state<Error> + sml::exception<_> [ GuardExceptionError{} ] / ActExceptionError{} = sml::state<Error>,
        sml::state<Map> + sml::exception<_> [ GuardExceptionMap{} ] / ActExceptionMap{} = sml::state<Error>,
        sml::state<Mission> + sml::exception<_> [ GuardExceptionMission{} ] / ActExceptionMission{} = sml::state<Error>,
        sml::state<Prep> + sml::exception<_> [ GuardExceptionPrep{} ] / ActExceptionPrep{} = sml::state<Error>,

        // Shutdown Guard and Action declarations
        *sml::state<Init> + sml::event<EvToShutdown> [ GuardToShutdown{} ] / ActToShutdown{} = sml::X,
        sml::state<Idle> + sml::event<EvToShutdown>  [ GuardToShutdown{} ] / ActToShutdown{} = sml::X,
        sml::state<Error> + sml::event<EvToShutdown> [ GuardToShutdown{} ] / ActToShutdown{} = sml::X,
        sml::state<Map> + sml::event<EvToShutdown> [ GuardToShutdown{} ] / ActToShutdown{} = sml::X,
        sml::state<Mission> + sml::event<EvToShutdown> [ GuardToShutdown{} ] / ActToShutdown{} = sml::X,
        sml::state<Prep> + sml::event<EvToShutdown> [ GuardToShutdown{} ] / ActToShutdown{} = sml::X
    ); // transition table
  } //  operator () override
}; // Robot Center
#endif
