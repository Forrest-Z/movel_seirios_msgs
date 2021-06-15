#include "map_splitter/map_splitter.h"

MapSplitter::MapSplitter()
: width_(100), height_(100), overlap_(10)
{
  loadParams();
  setupTopics();
}

void MapSplitter::loadParams()
{
  ros::NodeHandle np("~"); // private node handler

  if (np.hasParam("width"))
    np.getParam("width", width_);

  if (np.hasParam("height"))
    np.getParam("height", height_);

  if (np.hasParam("overlap"))
    np.getParam("overlap", overlap_);
}

void MapSplitter::setupTopics()
{
  split_map_srv_ = nh_.advertiseService("split_map", &MapSplitter::splitMapSrvCb, this);
}

bool MapSplitter::splitMapSrvCb(movel_seirios_msgs::StringTrigger::Request &req,
                                movel_seirios_msgs::StringTrigger::Response &res)
{
  // deduce filenames
  std::string stem_fname;
  std::string img_fname;
  std::string yaml_fname;

  // find stem name (request should be able to accept stem, pgm, or yaml, but in absolute path)
  std::string key (".");
  std::size_t idx = req.input.rfind(key);
  if (idx != std::string::npos && idx >= req.input.size()-5)
  {
    stem_fname = req.input.substr(0, idx);
  }
  else
  {
    stem_fname = req.input;
  }
  img_fname = stem_fname + ".pgm";
  yaml_fname = stem_fname + ".yaml";

  // load input image
  cv::Mat bigmap = cv::imread(img_fname);
  if (bigmap.empty())
  {
    ROS_ERROR("Failed to open %s", img_fname.c_str());
    res.success = false;
    return false;
  }

  ROS_INFO("Loaded image of size %d, %d", bigmap.rows, bigmap.cols);

  // load input yaml
  YAML::Node mapspec;
  try
  {
    mapspec = YAML::LoadFile(yaml_fname); 
    YAML::Emitter out;
    out << mapspec;
    ROS_INFO("map spec\n%s", out.c_str());
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    ROS_ERROR("Failed to open %s", yaml_fname.c_str());
    res.success = false;
    return false;
  }

  // find bounds of valid data; two pairs of (row, col) bounding rectangle with valid map data
  // valid map means it isn't all unknown

  // split valid map into pieces, record its corresponding yaml spec

  res.success = true;
  return true;
}