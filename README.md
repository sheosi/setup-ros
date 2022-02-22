# setup-ros
An script automating the installation of ROS, with both ROS1 and ROS2

If you want to tweak what setup_ros installs change the variable `features`:

* `ros1`: ROS1 libs, enablement, and `catkin_ws` setup.
* `ros2`: ROS2 libs, enablement, and `dev_ws` setup.
* `nao`: Nao-related libs, needs `ros1` otherwise does nothing, only for `melodic`.
* `vscode`: Setup VSCode and needed extensions, also setups ROS2 workspace and config for ROS if `ros2` is enabled.

If both `ros1` and `ros2` are installed `ros_bridge` will be installed. Also, you will need to do `enable_ros1` or `enable_ros2` to load the respective environment. If only one is installed it will be available right away.
