#include <robot_center/robot_center.hpp>

/*
void signalHandler( int signum )
{
  LOG(INFO) << "SIGINT triggered.";
  g3::internal::shutDownLogging();
  exit(signum);

*/

int main(int argc, char** argv)
{
  // Argument Parsing
  std::string homeDir = getHomeDir();
  std::string file_path = homeDir;
  int maxMsgSize = 2048;

  cxxopts::Options options(argv[0], "Robot Center for managing the states of ROS");

  options.add_options()("l,logsize", "Maximum size of log message", cxxopts::value<int>()->default_value("2048"))(
      "p,path", "File path for log files", cxxopts::value<std::string>()->default_value(homeDir))("h,help", "Print "
                                                                                                            "usage");
  try
  {
    auto result = options.parse(argc, argv);
    if (result.count("help"))
    {
      std::cout << options.help() << std::endl;
      exit(0);
    }
    maxMsgSize = result["logsize"].as<int>();
    file_path = result["path"].as<std::string>();
    if (!dir_exists(file_path))
      throw("%s for log files does not exists", file_path.c_str());
  }
  catch (const cxxopts::OptionException& e)
  {
    std::cout << "[" << argv[0] << "]" << e.what() << std::endl;
    exit(1);
  }
  catch (const char* msg)
  {
    std::cout << msg << std::endl;
    exit(1);
  }

  // Init Logging
  auto worker = g3::LogWorker::createLogWorker();
  auto defaultHandler = worker->addDefaultLogger(argv[0], file_path);
  g3::only_change_at_initialization::setMaxMessageSize(maxMsgSize);
  g3::initializeLogging(worker.get());

  // TODO
  // Setting FatalPreLogging after initializeLogging as initializeLogging resets FatalPreLogging
  // g3::setFatalPreLoggingHook([]{ cleanup(); });

  LOG(INFO) << "Logging initialized with " << maxMsgSize << " msg size.";

  // Init SM

  // TODO Figure out SML Logger
  // Logger l;
  // sml::sm<Machine, sml::thread_safe<std::mutex>, sml::logger<Logger>> sm(l);
  sml::sm<RobotSM, sml::thread_safe<std::mutex>> sm;
  LOG(INFO) << "State Machine initialized with mutex ";
  //  sml.process_event(EvToIdle{});

  while (true)
  {
    // signal(SIGINT, signalHandler);
    // sm.visit_current_states([](auto state) { std::cout << state.c_str() << std::endl; });
    if (sm.is(sml::X))
    {
      LOG(INFO) << "Shutting Down due to SM having exited";
      g3::internal::shutDownLogging();
      break;
    }
  }
  return 0;
}
