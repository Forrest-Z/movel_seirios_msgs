//ENSURE IT IS SQUARE GRID

#define X_END 20.0
#define X_START -20.0

#define Y_END 20.0
#define Y_START -20.0

#define Z_END 1.0
#define Z_START 0.4

#define POINT_THRESHOLD 5

#define RESOLUTION 0.2

#define ROTATION_REQUIRED false

/*
IF ROTATION_REQUIRED == true 
//USE THIS COMMAND TO PUBLISH ROTATION IN DEGREES

'''''''''
rostopic pub /roll_pitch_yaw std_msgs/Float32MultiArray '{data: [0.0,0.0,0.0]}' --once
'''''''''

'''''''''''''''
stic_transform_publisher -20 -20 0 0 0 0 1 /rslidar  /local_map
'''''''''''''''

 */


