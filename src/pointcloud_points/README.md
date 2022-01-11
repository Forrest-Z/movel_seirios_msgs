## Pointcloud follower ##

### Designed by Movel AI

This package contains of three separate nodes:
1. *cloud_transposer.cpp* : program to switch height and width of a pointcloud
2. *points_follower.cpp* : C++ version of pointcloud follower that change the data type of a pointcloud data from sensor_msgs/PointCloud2 to movel_seirios_msgs/movelPointcloud2
3. *follower.py* : Python version of pointcloud follower. But depreciated because there is a bug on passing pcl in a map frame. Besides that, It is slower and requires more CPU resources.