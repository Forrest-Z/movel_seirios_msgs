# Speed limit zones
Configure areas where the robot must slow down.

## Usage
- Main launch file `speed_limit_zone.launch`
- Test file #1 `test_limit_speed.py`. Solely tests the limit_robot_speed service. Can be run without zone. Test it by running the file while the robot is moving and robot should slow down.
- Test file #2 `draw_one_zone.py`. Draws one speed limit zone and give a throttle %. Run the robot afterwards, should observe that when the robot is in the zone, it should slow down.

## Nodes
- mongo_bridge: Node to get speed zone data of selected map from MongoDB database
- speed_limit_zones: Node to draw zones and check if the robot is in the zone. Algorithm here: https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/#:~:text=1)%20Draw%20a%20horizontal%20line,true%2C%20then%20point%20lies%20outside.
- throttle_speed: Node to execute slowing down of robot if the robot lands in the zone. Does so by dynamically reconfiguring max_vel_x and max_vel_theta in the local planner

## Services
- [mongo_bridge] mongo_bridge/get_speed_zones: Service to get speed zone data from MongoDB database
- [speed_limit_zones] reduce_speed_zone: Service to draw the polygon and give a throttle %. 
- [throttle_speed] limit_robot_speed: Service to slow down robot speed by throttle % when called.

