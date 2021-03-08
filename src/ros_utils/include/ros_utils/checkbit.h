#ifndef BITCHECK_H
#define BITCHECK_H

#include <ros/ros.h>
#include <string>
#include <movel_msgs/GenericState.h>

struct BitLogger
{
  /*
  static constexpr uint8_t mask8_0 { 1 << 0 };
  static constexpr uint8_t mask8_1 { 1 << 1 };
  static constexpr uint8_t mask8_2 { 1 << 2 };
  static constexpr uint8_t mask8_3 { 1 << 3 };
  static constexpr uint8_t mask8_4 { 1 << 4 };
  static constexpr uint8_t mask8_5 { 1 << 5 };
  static constexpr uint8_t mask8_6 { 1 << 6 };
  static constexpr uint8_t mask8_7 { 1 << 7 };

  std::vector<uint8_t> vectorOf8Masks { mask8_0, mask8_1, mask8_2, mask8_3, mask8_4, mask8_5, mask8_6, mask8_7 };

  static constexpr uint16_t mask16_0 { 1 << 0 };
  static constexpr uint16_t mask16_1 { 1 << 1 };
  static constexpr uint16_t mask16_2 { 1 << 2 };
  static constexpr uint16_t mask16_3 { 1 << 3 };
  static constexpr uint16_t mask16_4 { 1 << 4 };
  static constexpr uint16_t mask16_5 { 1 << 5 };
  static constexpr uint16_t mask16_6 { 1 << 6 };
  static constexpr uint16_t mask16_7 { 1 << 7 };
  static constexpr uint16_t mask16_8 { 1 << 8 };
  static constexpr uint16_t mask16_9 { 1 << 9 };
  static constexpr uint16_t mask16_10 { 1 << 10 };
  static constexpr uint16_t mask16_11 { 1 << 11 };
  static constexpr uint16_t mask16_12 { 1 << 12 };
  static constexpr uint16_t mask16_13 { 1 << 13 };
  static constexpr uint16_t mask16_14 { 1 << 14 };
  static constexpr uint16_t mask16_15 { 1 << 15 };

  std::vector<uint16_t> vectorOf16Masks { mask16_0, mask16_1, mask16_2, mask16_3, mask16_4, mask16_5, mask16_6,
                                                mask16_7, mask16_8, mask16_9, mask16_10, mask16_11, mask16_12,
                                                mask16_13, mask16_14, mask16_15 };
  */

  void checkFalse(movel_msgs::GenericState& log_state, const uint8_t& data, const std::vector<std::string>& log_code)
  {
    for (int i = 0; i < log_code.size(); ++i)
    {
      movel_msgs::KeyValue log;
      log.key = log_code[i];
      log.value = "False";
      uint8_t mask{ 1 << i };
      if (data & mask)
        log_state.states.push_back(log);
    }
  }

  void checkTrue(movel_msgs::GenericState& log_state, const uint8_t& data, const std::vector<std::string>& log_code)
  {
    for (int i = 0; i < log_code.size(); ++i)
    {
      movel_msgs::KeyValue log;
      log.key = log_code[i];
      log.value = "True";
      uint8_t mask{ 1 << i };
      if (data & mask)
        log_state.states.push_back(log);
    }
  }

  void checkFalse(movel_msgs::GenericState& log_state, const uint16_t& data, const std::vector<std::string>& log_code)
  {
    for (int i = 0; i < log_code.size(); ++i)
    {
      movel_msgs::KeyValue log;
      log.key = log_code[i];
      log.value = "False";
      uint16_t mask{ 0 << i };
      if (data & mask)
        log_state.states.push_back(log);
    }
  }

  void checkTrue(movel_msgs::GenericState& log_state, const uint16_t& data, const std::vector<std::string>& log_code)
  {
    for (int i = 0; i < log_code.size(); ++i)
    {
      movel_msgs::KeyValue log;
      log.key = log_code[i];
      log.value = "True";
      uint16_t mask{ 1 << i };
      if (data & mask)
        log_state.states.push_back(log);
    }
    /*
    for (int i = 0; i < vectorOf16Masks.size(); ++i)
    {
      movel_msgs::KeyValue log;
      log.key = log_code[i];
      log.value = "True";
      if (data & vectorOf16Masks[i])
        log_state.states.push_back(log);
    }
    */
  }

  // Can't pass in a vector as a template parameter :(
  /*
  template <class T, movel_msgs::GenericState, std::vector<std::string>>
  void checkTrue(movel_msgs::GenericState &log_state, T &data, const std::vector<std::string> &log_code)
  {
    for (int i = 0; i < log_code.size(); ++i)
    {
      movel_msgs::KeyValue log;
      constexpr T mask { 1 << i };
      log.key = log_code[i];
      log.value = "True";
      if (data & mask)
        log_state.states.push_back(log);
    }
  }
  */
};
#endif
