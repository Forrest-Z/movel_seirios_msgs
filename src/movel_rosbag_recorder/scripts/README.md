# recorder_node.py

**Initializing the recorder via topic `/init_recorder`**    
* `/init_recorder` takes in an std_msgs/String msg    
* The payload is a JSON-like string with the desired settings for the rosbag recording 

**Parameters**    
* `bag_name`: Name of output bag    
    Note that when param `split_duration_secs` is enabled, the splits will be index as `/.../bag_name_0.bag, /.../bag_name_2.bag, etc`
* `topics` (optional): List of topics to record    
    If not specified, all topics will be recorded    
    All given topic names will be renamed/standardized to `/topic_name` regardless of whether or not the `'/'` prefix was present in the input list
* `split_duration_secs` (optional): Split recording such that each bag has a max duration of `split_duration_secs`. 

* Full payload example  
    ```
    std_msgs/String instance

    data: '{
        "bag_name": "/home/movel/rosbags/recorder_test/bag_name_topics_split_duration_secs_params",
        "topics": ["/map", "/pose_stamped", "tf", "scan", "/move_base/TebLocalPlannerROS/local_plan", "/move_base/TebLocalPlannerROS/global_plan"],
        "split_duration_secs": 35.5
    }' 

    ** Note that the quotes have to be properly escaped
    ** Using a JSON encoding/decoding library is recommended
    ```
