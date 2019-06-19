# Rviz GUI Communication Protocol

### GUI to Drone
#### /target_pose
Type:geometry_msgs/PoseStamped; 
Description: used for target pose for drone;
#### /flight_cmd
Type:std_msgs/Int16
Description: send takeoff and land command to drone.



### Drone to GUI
- /demo/state
Type: std_msgs/Int16
Description: display drone state to GUI.

### Rviz
use the /rviz/demo.rviz