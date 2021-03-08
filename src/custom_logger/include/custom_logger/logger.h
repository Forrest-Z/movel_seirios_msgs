#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <iostream>
#include <unistd.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include "ros/ros.h"

using namespace std;

typedef boost::function<void(const string &)> MsgCallback;

class Logger
{
public:
  void log_default(const std::string &msg)
  {
    cout << msg << endl;
  }

  MsgCallback warn;
  MsgCallback debug;
  MsgCallback info;
  MsgCallback error;

  Logger()
  {
    warn = boost::bind(&Logger::log_default, this, _1);
    debug = boost::bind(&Logger::log_default, this, _1);
    info = boost::bind(&Logger::log_default, this, _1);
    error = boost::bind(&Logger::log_default, this, _1);
  }
};

class ROSLogger : public Logger
{
public:
  void handleWarnMessages(const std::string &msg) { ROS_WARN("%s", msg.c_str()); }
  void handleDebugMessages(const std::string &msg) { ROS_DEBUG("%s", msg.c_str()); }
  void handleInfoMessages(const std::string &msg) { ROS_INFO("%s", msg.c_str()); }
  void handleErrorMessages(const std::string &msg) { ROS_ERROR("%s", msg.c_str()); }

  ROSLogger()
  {
    warn = boost::bind(&ROSLogger::handleWarnMessages, this, _1);
    debug = boost::bind(&ROSLogger::handleDebugMessages, this, _1);
    info = boost::bind(&ROSLogger::handleInfoMessages, this, _1);
    error = boost::bind(&ROSLogger::handleErrorMessages, this, _1);
  }
};

#endif
