# bt_patrol_NIRA
Thia package is a recopilation for bt formulation of patrol robot, with first porpose of detect persons.

## To run a demo of the process, it is necessary to execute the following steps, avoiding simulation errors.
1. ros2 launch gazebo_ros gzserver.launch.py world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world
2. ros2 launch gazebo_ros gzclient.launch.py 
3. ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=true
4. ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:=-2.0 y_pose:=-0.5
5. ros2 launch nav2_bringup rviz_launch.py rviz_config:=/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
6. ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml
