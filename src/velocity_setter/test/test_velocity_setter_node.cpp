#include <velocity_setter/velocity_setter.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

class VelocitySetterNodeTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      srv_server = nh_.advertiseService("/move_base/TebLocalPlannerROS/set_parameters", &VelocitySetterNodeTest::dyn_rec, this);
    }
    ros::ServiceServer srv_server;
    ros::NodeHandle nh_;
    std::string service_name = "/move_base/TebLocalPlannerROS/set_parameters";
    std::string res_name;
    double res_val;

  public:
    bool dyn_rec(dynamic_reconfigure::Reconfigure::Request &req, dynamic_reconfigure::Reconfigure::Response &res)
      {
        res_name = req.config.doubles[0].name;
        res_val = req.config.doubles[0].value;
      }
};

TEST_F(VelocitySetterNodeTest, testNotRunningClient)
{
  VelocitySetter vs;
  bool test_velocity = vs.setVelocity(1);
  EXPECT_FALSE(test_velocity);
}

// init service client, checks whether name & value of params
TEST_F(VelocitySetterNodeTest, testRunningClient)
{
  VelocitySetter vs;
  vs.set_client_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(service_name);
  ros::ServiceClient srv_client = nh_.serviceClient<movel_seirios_msgs::SetVelocity>("velocity_setter_node/set_velocity");
  srv_client.waitForExistence();

  movel_seirios_msgs::SetVelocity test_set_vel;
  test_set_vel.request.name = "cleaning";
  srv_client.call(test_set_vel);

  EXPECT_EQ(res_name, "max_vel_x");
  EXPECT_EQ(res_val, 0.3);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_velocity_setter_node");
  ::testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}