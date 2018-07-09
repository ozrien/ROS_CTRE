# ROS_CTRE

On start up, the pi will run joy_diff.launch which calls diff_drive_test.launch and teleop_twist_joy.launch as well as setting up the canable. If the joystick is plugged into a usb hub instead of directly into the pi on startup, the joystick will probably not be found for unknown reasons. This can be fixed by plugging the joystick into the pi directly. This start up launch is sufficient to drive the robot around, view twist, and view lidar. To kill this process, run `sudo systemctl stop launch_base_control.service`. 

`source ~/Documents/CTRE/ROS_CTRE/ctreROS_ws/devel/setup.bash` must be run to use any ros commands in that terminal instance. 
The recommended terminal program for ros use is terminator. In ubuntu mate it can be found under Applications->System Tools->Terminator. It allows for multiple terminal sessions in the same window and other features. `man terminator` for details.
All of the launch files are under the ros_control_boilerplate pkg. To run a launch file use `roslaunch pkg launch_file` replacing pkg and launch_file with the desire targets. Tab completion works here like for most ros commands. This project is set up so that pretty much every demo can be run using a launch file.

Launch files:
joy_diff.launch - runs diff_drive_test.launch and teleop_twist_joy.launch. Specifies a can interface of can0 (canable).
diff_drive_test.launch - runs the hardware node including the drive train controller. A can interface MUST be specified using the command line argument `interface:=interface_name` replacing interface_name with the name of the interface (bug).
teleop_twist_joy.launch - runs the joystick node and the node which converts joystick data to a twist message for controlling the robot.
teleop_twist_keyboard.launch - runs a node which publishes twist values. (note, for the robot to be enabled, a joystick msg with button 4 pressed must be sent)
rviz_lidar.launch - runs an rviz display of the lidar (note, lidar must be plugged in and then `sudo chmod a+rw /dev/ttyUSB0` must be run prior to use)
rviz_twist.launch - runs an rviz display of twist (note twist must be published for this to work, plugging in the joystick isn't sufficient)
rviz_lidar_twist.launch - runs an rviz display of the lidar and twist in the same session


 
