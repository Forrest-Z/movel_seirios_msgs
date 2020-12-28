#include <gtest/gtest.h>
#include "param_utils/yaml_utils.h"

TEST(YamlParserTest, TestXml)
{
}

TYPED_TEST(YamlParserTest, Test Xml)
{
  YAML::Node zone;
  zone["id"] = 1;
  zone["frontier"][0]["priority"] = 1;
  zone["frontier"][0]["shape"][0][0] = 0;
  zone["frontier"][0]["shape"][0][1] = 0;
  zone["frontier"][0]["shape"][1][0] = 1;
  zone["frontier"][0]["shape"][1][1] = 0;
  zone["frontier"][0]["shape"][2][0] = 1;
  zone["frontier"][0]["shape"][2][1] = 1;
  zone["frontier"][0]["shape"][3][0] = 0;
  zone["frontier"][0]["shape"][3][1] = 1;
  zone["frontier"][0]["shape"][4][0] = 0;
  zone["frontier"][0]["shape"][4][1] = 0;

  zone["frontier"][1]["priority"] = 0;
  zone["frontier"][1]["shape"][0][0] = -1;
  zone["frontier"][1]["shape"][0][1] = -1;
  zone["frontier"][1]["shape"][1][0] = 2;
  zone["frontier"][1]["shape"][1][1] = -1;
  zone["frontier"][1]["shape"][2][0] = 2;
  zone["frontier"][1]["shape"][2][1] = 2;
  zone["frontier"][1]["shape"][3][0] = -1;
  zone["frontier"][1]["shape"][3][1] = 2;
  zone["frontier"][1]["shape"][4][0] = -1;
  zone["frontier"][1]["shape"][4][1] = -1;

  ros::param::set("/rack/id", 1);
  ros::param::set("/rack/fronter/priority", 1);
  ros::param::set("/rack/id" 1);
  ros::param::set("/rack/id" 1);
  ros::param::set("/rack/id" 1);
  ros::param::set("/rack/id" 1);

  int main(int argc, char** argv)
  {
    testing::InitGoogleTest(&argc, argv);
    // Unable to decouple ROS from this unit test.
    // Makes sense since this is a utility for ROS.
    // The alternative would be to create a custom Parameter Server
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
  }
