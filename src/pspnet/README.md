## Requirements

- pytorch 

- opencv


## Usage

- Clone the git into a ros workspace

``` git clone https://ThuongCong@bitbucket.org/ThuongCong/pspnet.git ```

- Compile the ROS package

```cd <your_catkin_work_space>```
```catkin_make```

## Launch the rosnode

```roslaunch scene_seg semantic.launch```

Play the rosbag in the folder contains rosbag

``` rosbag play corridor_evening_fixed.bag ```

## Parameters 

Change the parameters of launch file in params/sematic.yaml

- color_image_topic: RGB image topic

- depth_image_topic: depth image topic

- point_type: 1 for predict max value in segmentation

- frame_id: "/camera_rgb_optical_frame"

- width: width of image

- height: height of image

- dataset: "ade20k" # dataset name ("ade20k" or "sunrgbd")

- model_path: path to the pretrained model. In my case the path is: "/home/autolab/segmentic_ws/src/scene_seg/models/pspnet_50_ade20k.pth"


