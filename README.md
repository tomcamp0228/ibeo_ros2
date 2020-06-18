# Ibeo_ROS2 project

This is a project original built for [Ibeo 8l LUX LiDAR](https://www.ibeo-as.com) in ROS2 environment, the project is fully functional under [ROS 2 Eloquent Elusor](https://index.ros.org/).
Current project mainly has two functions:

- Get point cloud from the device and broadcast to ROS2 network;
- Read .idc file, which is a file format provided by Ibeo to record data under Windows system.
  
**Before starting, check your Ibeo LiDAR IP address and port/you .idc location and adjust them in the `src/ibeo_8l_client/launch/para.yaml`**

To briefly run the program, follow the instructions below:

  1. `cd ${Your workspace}`
  2. `source /opt/ros/eloquent/setup.bash`
  3. `colcon build` **OR** `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes` ( to generate compile commands for vscode intellisense )
  4. `source install/setup.bash`
  5. `ros2 launch ibeo_8l_client live.launch.py` (for live) **OR** `ros2 launch ibeo_8l_client file_sim.launch.py` (for .idc file)

Thanks to my teammates in graduation project and teammates in our group.