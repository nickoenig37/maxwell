## Running Sim 
### 1. Run the following commands in separate terminals

#### Terminal 1- simulation
```
    ros2 launch nav2_gps_waypoint_follower_demo gps_waypoint_follower.launch.py use_rviz:=True
```
#### Terminal 2- mapviz
```
    ros2 launch nav2_gps_waypoint_follower_demo mapviz.launch.py
```
#### Terminal 3- waypoint follower for mapviz (so you can click to where you want to go to)
```
    ros2 run nav2_gps_waypoint_follower_demo interactive_waypoint_follower
```
#### Terminal 4- running my node 
```
    ros2 run rover_heading heading_node
```

