#include "map_splitter/map_splitter.h"

MapSplitter::MapSplitter()
: width_(100), height_(100), overlap_(10), scale_(2)
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

  if (np.hasParam("scale"))
    np.getParam("scale", scale_);
}

void MapSplitter::setupTopics()
{
  split_map_srv_ = nh_.advertiseService("split_map", &MapSplitter::splitMapSrvCb, this);
}

void MapSplitter::getMapBounds(cv::Mat &bigmap, int &top, int &left, int &bottom, int &right)
{
  // find bounds of valid data; two pairs of (row, col) bounding rectangle with valid map data
  // valid map means it isn't all unknown
  
  double minval, maxval;
  cv::Point minidx, maxidx;

  // top bound
  // ROS_INFO("looking for top bound");
  for (int i = 0; i < bigmap.rows; i++)
  {
    cv::minMaxLoc(bigmap.row(i), &minval, &maxval, &minidx, &maxidx);
    // ROS_INFO("%d, min %5.2f, max %5.2f", i, minval, maxval);
    if (minval != maxval)
    {
      // ROS_INFO("top bound %d with value %5.2f", i, minval);
      top = i;
      break;
    }
  }

  // bottom bound
  // ROS_INFO("looking for bottom bound %d", bigmap.rows);
  for (int i = bigmap.rows-1; i >= 0; i--)
  {
    cv::minMaxLoc(bigmap.row(i), &minval, &maxval, &minidx, &maxidx);
    // ROS_INFO("%d, min %5.2f, max %5.2f", i, minval, maxval);
    if (minval != maxval)
    {
      // ROS_INFO("bottom bound %d with value %5.2f", i, minval);
      bottom = i;
      break;
    }
  }

  // left bound
  // ROS_INFO("looking for left bound");
  for (int i = 0; i < bigmap.cols; i++)
  {
    cv::minMaxLoc(bigmap.col(i), &minval, &maxval, &minidx, &maxidx);
    // ROS_INFO("%d, min %5.2f, max %5.2f", i, minval, maxval);
    if (minval != maxval)
    {
      // ROS_INFO("left bound %d with value %5.2f", i, minval);
      left = i;
      break;
    }
  }

  // right bound
  // ROS_INFO("looking for right bound");
  for (int i = bigmap.cols-1; i >= 0; i--)
  {
    cv::minMaxLoc(bigmap.col(i), &minval, &maxval, &minidx, &maxidx);
    // ROS_INFO("%d, min %5.2f, max %5.2f", i, minval, maxval);
    if (minval != maxval)
    {
      // ROS_INFO("right bound %d with value %5.2f", i, minval);
      right = i;
      break;
    }
  }
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
  mkdir(stem_fname.c_str(), 0775);

  // load input image
  cv::Mat bigmap = cv::imread(img_fname, cv::IMREAD_GRAYSCALE);
  if (bigmap.empty())
  {
    ROS_ERROR("Failed to open %s", img_fname.c_str());
    res.success = false;
    return false;
  }

  ROS_INFO("Loaded image of size %d, %d, %d", bigmap.rows, bigmap.cols, bigmap.dims);

  // load input yaml
  YAML::Node bigmap_spec;
  try
  {
    bigmap_spec = YAML::LoadFile(yaml_fname); 
    YAML::Emitter out;
    out << bigmap_spec;
    ROS_INFO("map spec\n%s", out.c_str());
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    ROS_ERROR("Failed to open %s", yaml_fname.c_str());
    res.success = false;
    return false;
  }

  // find map bounds
  int top, bottom, left, right;
  getMapBounds(bigmap, top, left, bottom, right);
  ROS_INFO("bound is (%d, %d), (%d, %d)", top, left, bottom, right);
  cv::Mat validmap(bigmap, cv::Rect(cv::Point(left, top), cv::Point(right, bottom)));
  YAML::Node validmap_spec = YAML::Clone(bigmap_spec);

  double resolution = bigmap_spec["resolution"].as<double>();

  validmap_spec["origin"][0] = bigmap_spec["origin"][0].as<double>() + left * resolution;
  validmap_spec["origin"][1] = bigmap_spec["origin"][1].as<double>() + left * resolution;
  validmap_spec["name"] = bigmap_spec["name"].as<std::string>() + " (tight)";

  std::string validmap_fname, validmap_spec_fname;
  validmap_fname = stem_fname + "/valid.pgm";
  validmap_spec_fname = stem_fname + "/valid.yaml";
  validmap_spec["image"] = validmap_fname;
  cv::imwrite(validmap_fname, validmap);
  std::ofstream validmap_out(validmap_spec_fname.c_str());
  validmap_out << validmap_spec;
  validmap_out.close();

  cv::imshow("blah", validmap);

  // scale map, save with adjusted spec
  // cv::Mat smallmap (validmap.rows, validmap.cols, validmap.type(), 0);
  cv::Mat smallmap;
  scaleMap(validmap, smallmap, scale_);
  ROS_INFO("scaleMap OK");
  std::string smallmap_fname = stem_fname + "/scaled.pgm";
  cv::imwrite(smallmap_fname, smallmap);

  YAML::Node smallmap_spec = YAML::Clone(validmap_spec);
  smallmap_spec["image"] = smallmap_fname;
  smallmap_spec["resolution"] = resolution * scale_;
  smallmap_spec["name"] = bigmap_spec["name"].as<std::string>() + " (scaled)";
  std::string smallmap_spec_fname = stem_fname + "/scaled.yaml";
  std::ofstream smallmap_out(smallmap_spec_fname.c_str());
  smallmap_out << smallmap_spec;
  smallmap_out.close();
    
  // split valid map into pieces, record its corresponding yaml spec
  // also, keep track of the transitions

  // calculate rows and columns
  int split_rows = validmap.rows/(height_ - overlap_);
  int split_cols = validmap.cols/(width_ - overlap_);

  YAML::Node transitions; // for transitions between map pieces

  for (int i = 0; i < split_rows; i++)
  {
    for (int j = 0; j < split_cols; j++)
    {
      top = i*height_;
      if (i > 0)
        top -= overlap_;
      left = j*width_;
      if (j > 0)
        left -= overlap_;

      bottom = top + height_;
      if (i == split_rows-1)
        bottom = validmap.rows-1;
      right = left + width_;
      if (j == split_cols-1)
        right = validmap.cols-1;

      std::string piece_suffix = std::to_string(i) + "_" + std::to_string(j);

      cv::Mat piece(validmap, cv::Rect(cv::Point(left, top), cv::Point(right, bottom)));
      std::string fname_ij = stem_fname + "/" + piece_suffix + ".pgm";
      cv::imwrite(fname_ij, piece);
      ROS_INFO("write %s %d, %d", fname_ij.c_str(), i, j);
      cv::imshow(fname_ij, piece);

      YAML::Node piece_spec = YAML::Clone(validmap_spec);
      piece_spec["image"] = fname_ij;
      double piece_ox, piece_oy;
      piece_ox = validmap_spec["origin"][0].as<double>() + left * resolution;
      piece_oy = validmap_spec["origin"][1].as<double>() + top * resolution;
      piece_spec["origin"][0] = piece_ox;
      piece_spec["origin"][1] = piece_oy;
      piece_spec["name"] = bigmap_spec["name"].as<std::string>() + " " + piece_suffix;
      std::string piece_spec_fname = stem_fname + "/" + piece_suffix + ".yaml";
      std::ofstream piece_out(piece_spec_fname);
      piece_out << piece_spec;
      piece_out.close();

      // setup transitions
      YAML::Node transspec_ij;
      transspec_ij["width"] = resolution * width_;
      transspec_ij["height"] = resolution * height_;
      YAML::Node origin_pt;
      origin_pt["x"] = piece_ox;
      origin_pt["y"] = piece_oy;
      transspec_ij["origin"] = origin_pt;

      // N
      if (i > 0)
      {
        YAML::Node transition;
        std::string dst_name = std::to_string(i-1) + "_" + std::to_string(j);
        transition["dst"] = dst_name;

        YAML::Node pt0;
        pt0["x"] = piece_ox + resolution * (width_ - overlap_);
        pt0["y"] = piece_oy + 0.5 * resolution * overlap_;
        transition["pt0"] = pt0;

        YAML::Node pt1;
        pt1["x"] = piece_ox + resolution * overlap_;
        pt1["y"] = piece_oy + 0.5 * resolution * overlap_;
        transition["pt1"] = pt1;

        transspec_ij["transitions"].push_back(transition);
      }

      // NW
      if (i > 0 && j > 1)
      {
        YAML::Node transition;
        std::string dst_name = std::to_string(i-1) + "_" + std::to_string(j-1);
        transition["dst"] = dst_name;

        YAML::Node pt0;
        pt0["x"] = piece_ox + resolution * overlap_;
        pt0["y"] = piece_oy + 0.5 * resolution * overlap_;
        transition["pt0"] = pt0;

        YAML::Node pt1;
        pt1["x"] = piece_ox + 0.5 * resolution * overlap_;
        pt1["y"] = piece_oy + resolution * overlap_;
        transition["pt1"] = pt1;

        transspec_ij["transitions"].push_back(transition);
      }

      // W
      if (j > 1)
      {
        YAML::Node transition;
        std::string dst_name = std::to_string(i) + "_" + std::to_string(j-1);
        transition["dst"] = dst_name;

        YAML::Node pt0;
        pt0["x"] = piece_ox + 0.5 * resolution * overlap_;
        pt0["y"] = piece_oy + resolution * overlap_;
        transition["pt0"] = pt0;

        YAML::Node pt1;
        pt1["x"] = piece_ox + 0.5 * resolution * overlap_;
        pt1["y"] = piece_oy + (height_ - overlap_) * resolution;
        transition["pt1"] = pt1;

        transspec_ij["transitions"].push_back(transition);
      }

      // SW
      if (j > 1 && i < split_rows - 1)
      {
        YAML::Node transition;
        std::string dst_name = std::to_string(i+1) + "_" + std::to_string(j-1);
        transition["dst"] = dst_name;

        YAML::Node pt0;
        pt0["x"] = piece_ox + 0.5 * resolution * overlap_;
        pt0["y"] = piece_oy + (height_ - overlap_) * resolution;
        transition["pt0"] = pt0;

        YAML::Node pt1;
        pt1["x"] = piece_ox + resolution * overlap_;
        pt1["y"] = piece_oy + (height_ - 0.5*overlap_) * resolution;
        transition["pt1"] = pt1;

        transspec_ij["transitions"].push_back(transition);
      }

      // S
      if (i < split_rows - 1)
      {
        YAML::Node transition;
        std::string dst_name = std::to_string(i+1) + "_" + std::to_string(j);
        transition["dst"] = dst_name;

        YAML::Node pt0;
        pt0["x"] = piece_ox + resolution * overlap_;
        pt0["y"] = piece_oy + (height_ - 0.5*overlap_) * resolution;
        transition["pt0"] = pt0;

        YAML::Node pt1;
        pt1["x"] = piece_ox + (width_ - overlap_) * resolution;
        pt1["y"] = piece_oy + (height_ - 0.5*overlap_) * resolution;
        transition["pt1"] = pt1;

        transspec_ij["transitions"].push_back(transition);
      }

      // SE
      if (i < split_rows - 1 && j < split_cols -1)
      {
        YAML::Node transition;
        std::string dst_name = std::to_string(i+1) + "_" + std::to_string(j+1);
        transition["dst"] = dst_name;

        YAML::Node pt0;
        pt0["x"] = piece_ox + (width_ - overlap_) * resolution;
        pt0["y"] = piece_oy + (height_ - 0.5*overlap_) * resolution;
        transition["pt0"] = pt0;

        YAML::Node pt1;
        pt1["x"] = piece_ox + (width_ - 0.5*overlap_) * resolution;
        pt1["y"] = piece_oy + (height_ - overlap_) * resolution;
        transition["pt1"] = pt1;

        transspec_ij["transitions"].push_back(transition);
      }

      // E
      if (j < split_cols - 1)
      {
        YAML::Node transition;
        std::string dst_name = std::to_string(i) + "_" + std::to_string(j+1);
        transition["dst"] = dst_name;

        YAML::Node pt0;
        pt0["x"] = piece_ox + (width_ - 0.5*overlap_) * resolution;
        pt0["y"] = piece_oy + (height_ - overlap_) * resolution;
        transition["pt0"] = pt0;

        YAML::Node pt1;
        pt1["x"] = piece_ox + (width_ - 0.5*overlap_) * resolution;
        pt1["y"] = piece_oy + overlap_ * resolution;
        transition["pt1"] = pt1;

        transspec_ij["transitions"].push_back(transition);
      }

      // NE
      if (i > 0 && j < split_cols - 1)
      {
        YAML::Node transition;
        std::string dst_name = std::to_string(i-1) + "_" + std::to_string(j+1);
        transition["dst"] = dst_name;

        YAML::Node pt0;
        pt0["x"] = piece_ox + (width_ - 0.5*overlap_) * resolution;
        pt0["y"] = piece_oy + overlap_ * resolution;
        transition["pt0"] = pt0;

        YAML::Node pt1;
        pt1["x"] = piece_ox + (width_ - overlap_) * resolution;
        pt1["y"] = piece_oy + 0.5 * overlap_ * resolution;
        transition["pt1"] = pt1;

        transspec_ij["transitions"].push_back(transition);
      }

      transitions[piece_suffix] = transspec_ij;
    }
  }

  std::string transitions_fname = stem_fname + "/transitions.yaml";
  std::ofstream transitions_out(transitions_fname.c_str());
  transitions_out << transitions;
  transitions_out.close();
  
  cv::waitKey(0);
  cv::destroyAllWindows();
  res.success = true;
  return true;
}

void MapSplitter::scaleMap(cv::Mat &bigmap, cv::Mat &smallmap, int scale)
{
  int sc, sr;
  sc = bigmap.cols / scale;
  sr = bigmap.rows / scale;
  smallmap = cv::Mat(cv::Size(sc, sr), CV_8U, cv::Scalar(0));
  for (int i = 0; i < smallmap.rows; i++)
  {
    for (int j = 0; j < smallmap.cols; j++)
    {
      cv::Point top_left (j * scale, i * scale);
      cv::Point bottom_right ((j+ 1) * scale - 1, (i + 1) * scale - 1);
      double minv, maxv;
      cv::minMaxLoc(bigmap(cv::Rect(top_left, bottom_right)), &minv, &maxv);
      // ROS_INFO("scale map %d/%d, %d/%d, %5.2f, %5.2f", i, sr, j, sc, minv, maxv);

      smallmap.at<uint8_t>(i, j) = (uint8_t)minv;
    }
  }
  // ROS_INFO("scale OK");
}