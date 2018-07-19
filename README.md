# ROS_CTRE

On start up, the pi will run joy_diff.launch which calls diff_drive_test.launch and teleop_twist_joy.launch as well as setting up the canable. If the joystick is plugged into a usb hub instead of directly into the pi on startup, the joystick will probably not be found for unknown reasons. This can be fixed by plugging the joystick into the pi directly. This start up launch is sufficient to drive the robot around, view twist, and view lidar. To kill this process, run `sudo systemctl stop launch_base_control.service`. 

`source ~/Documents/CTRE/ROS_CTRE/ctreROS_ws/devel/setup.bash` must be run to use any ros commands in that terminal instance. 
The recommended terminal program for ros use is terminator. In ubuntu mate it can be found under Applications->System Tools->Terminator. It allows for multiple terminal sessions in the same window and other features. `man terminator` for details.
All of the launch files are under the ros_control_boilerplate pkg. To run a launch file use `roslaunch pkg launch_file` replacing pkg and launch_file with the desire targets. Tab completion works here like for most ros commands. This project is set up so that pretty much every demo can be run using a launch file. Any combination of launch files can be launched together, though some launch files use the same nodes and others requires some topics to be published to for them to do anything.

Launch files:
* joy_diff.launch - runs diff_drive_test.launch and teleop_twist_joy.launch. Specifies a can interface of can0 (canable).
* diff_drive_test.launch - runs the hardware node including the drive train controller. A can interface MUST be specified using the command line argument `interface:=interface_name` replacing interface_name with the name of the interface (at some point I will probably make it auto discover can interfaces correctly).
* teleop_twist_joy.launch - runs the joystick node and the node which converts joystick data to a twist message for controlling the robot. The controller must be in x mode.
* teleop_twist_keyboard.launch - runs a node which publishes twist values with keyboard control. (note, for the robot to be enabled, a joystick msg with button 4 pressed must be sent. This can be done using enable.sh under ctreROS_ws/src/ros_control_boilerplate/scripts/ Use of this enable script is vaguely unsafe)
* rviz_lidar.launch - runs an rviz display of the lidar (note, lidar must be plugged in and then `sudo chmod a+rw /dev/ttyUSB0` must be run prior to use. lidar_chmod.sh under ctreROS_ws/src/ros_control_boilerplate/scripts/ runs this command )
* rviz_twist.launch - runs an rviz display of twist (note twist must be published for this to work, plugging in the joystick isn't sufficient on its own)
* rviz_lidar_twist.launch - runs an rviz display of the lidar and twist in the same session

Some basic ROS commands:
* Topics can be published to using `rostopic pub topic_name` with the correct topic_name (`-r hz` will publish at the frequency hz)
* Topics can be listed using `rostopic list`. this is a useful way to see if a master is running.
* Topics can be subcribed to with `rostopic echo topic_name` with the correct topic_name

Other useful commands/demos:
* `vcanEnable.sh` enables a vcan0 interface (in ROS_CTRE directory).
* `canableStart.sh` enables a canable interface (can0) (in ROS_CTRE directory).
* See the README at https://github.com/rgreenblatt/candleLight_fw for canable firmware installation instructions.
* The enable.sh script under ctreROS_ws/src/ros_control_boilerplate/scripts/ publishes an enabling joystick msg. Vaguely unsafe.
* `rosrun rqt_graph rqt_graph` will display a node/subscriber/publisher diagram.
* `rosrun rqt_plot rqt_plot` may be used to plot numeric topics. (some topics which will work are `/ctrerobot/diff_drive_controller/cmd_vel/linear/x`, `/ctrerobot/talon_states/output_voltage[0]`, and `/ctrerobot/talon_states/position[1]`) 
* `candump interface,arbid:mask` will filter and output msgs on that interface. ie `candump can0,000401BF:1FFFFFFF`. vcan can be demonstated with this pretty well.
* Multi device procedure: extract ips of desired master and desire other device. Make sure both devices can ping each other. `export ROS_MASTER_URI=http://10.0.0.8:11311` on the other device replacing 10.0.0.8 with the master ip. Test this set up by with ros topic list and then by echoing a topic. See http://wiki.ros.org/ROS/Tutorials/MultipleMachines for troubleshooting. (I tested and this does work on the CTR network. This setup may not be stable on other networks, static ips are desireable to improve stability.

# Zebracorns
Special thanks to the [Zebracorns-FRC900](https://team900.org/) for contributing their talon interface ROS package and hardware interface node!
