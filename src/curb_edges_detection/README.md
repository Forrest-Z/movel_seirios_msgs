## Ramp Detection Algorithm

### Configuration

In task_supervisor.yaml, under pcl_localization_handler:
  pcl_localization_launch_file: "3D_terrain_move_base_graph.launch"

angle_th: Maximum slope for ramp to be considered traversable (rad)

By default, we merge the lidar and the camera point clouds because the lidar has 360 degree horizontal FOV and longer range while the camera is useful at where the lidar has blind spots.

Working with raw point clouds is costly due to the large number of datapoints. 
To be useful in realtime, we enhance the performance and speed of the algorithm by preprocessing the incoming stream of point cloud data, filtering on x, y and z axes, and downsampling.

In order to identify planes that may be ramps, we find the normals N for all the points in pointcloud P. Then we can take advantage of
a Region Growing algorithm to merge the points in P that are close enough in terms of the smoothness and curvature
constraints imposed on the point normals N. This is an important step because pointclouds may be noisy and that many outdoor ramps
often have coarse surfaces. 

In this implementation, we used a smoothness threshold of 5◦ and a curvature threshold of 40◦.

Then, we iteratively segment the largest planar component from the remaining cloud using RANSAC, and use coefficients of the fitted plane to calculate slope. 

If slope is greater than threshold, we add to output cloud. If less than threshold, we use k nearest neighbors to check if it might be a curb. If curb, we also add it to output cloud.

Just before we publish the output cloud, we set its frame id to be "base_link_flattened" though we had previously transformed it to base_link frame. This is because move_base assumes obstacles are within certain height of a robot's 2D navigation plane. 

The workaround is "flatten" the base_link frame into a fake "base_link_flattened" frame by removing z, roll and pitch channels from the base_link frame. The output pointcloud's frame is set to "base_link_flattened" to trick move_base into thinking the pointcloud is within the height of the 2D navigation plane.

## Room for Improvement

Curb detection is less consistent than ramp detection. Can consider having ramp detection separate from curb detection.
Typical spatial features of curb points are that the curb height is uniform in most cases and it is often at least 10cm higher than the ground. 
Also, the elevation changes sharply in the z axis. 
The curb points are usually smooth and continuous, and the curb often appears at the two sides of the road/carpark in most cases.
Based on these features, we propose the following algorithm in this paper to detect the curb points: https://ir.nsfc.gov.cn//paperDownload/1000013627952.pdf .

```
As shown in Algorithm 2, the curbs are represented by
two sets of points Ql and Qr . In this algorithm, we use a
dynamic sliding window to make the algorithm more time-
efficient and accurate. Due to the limited laser number and
field of view of the Velodyne sensor, we only concern about
the laser plane which is intact to cover the curb area. In order
to represent the roads, we fit the curbs using the parabola
model. The fitted curb points are represented by Dl and Dr .
The result of the curb detection algorithm is presented in
Fig. 5, the sparse curb points set is shown in square and
the dense curb points set is shown in green. The root mean
square errors (RMSEs) are calculated to evaluate the parabola
model for each side of the curb. In Fig. 5(a), the RMSEs are
respectively 0.0151 m and 0.0486 m for the left and right
side. In Fig. 5(b), the RMSEs are respectively 0.0345 m and
0.0241 m for the left and right side. In real-time experiments,
the average RMSE of each side is 0.0285 m
```