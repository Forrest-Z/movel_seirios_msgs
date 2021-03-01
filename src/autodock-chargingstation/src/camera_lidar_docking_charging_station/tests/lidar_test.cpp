#include <iostream>
#include <math.h>
#include "gtest/gtest.h"
#include<pthread.h>
#include<lidar_docking_/LidarDocking.hpp>
#include <bits/stdc++.h>
using namespace std;

struct LidarDockingTest : public testing::Test, public lidar_docking::LidarDocking{
protected:
    LidarDockingTest()
    {

    }

};


TEST_F(LidarDockingTest, computeMean)
{
    std::vector<int> arr {1,2,3,4,5,6};
    int min_elt = *min_element(arr.begin(), arr.end());
    int max_elt = *max_element(arr.begin(), arr.end());
    EXPECT_LE (computeMean(arr), max_elt);
    EXPECT_GE (computeMean(arr), min_elt);

}


TEST_F(LidarDockingTest, findLength)
{

    vector<pcl::PointXYZI> cluster;
    for(int i = 0; i< 10; i++)
    {
        pcl::PointXYZI pt;
        pt.x = i;
        pt.y = i/2;
        pt.z = 0;
        pt.intensity = 200;
        cluster.push_back(pt);
    }
    EXPECT_GE (find_length(cluster), 0);

}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "crop_map_test");
    return RUN_ALL_TESTS();
}

