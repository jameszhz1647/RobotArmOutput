# RobotArmOutput

**open the Rviz and joint_publisher_gui:**

​	`roslaunch panda_moveit_config demo.launch use_gui:=true`

**run moveIt move_group to execute motion plan:**

​	`rosrun move_group_pkg move_group_test.py` 

**run helper node to setup a joints data(position) publisher callback service:**

​	`	rosrun move_group_pkg talker.py` 

**record output data:**

​	`	rosbag record joints_data/position`

**convert rosbag to csv:**

​	instruction detail: https://github.com/AtsushiSakai/rosbag_to_csv

​	`	rosrun rosbag_to_csv rosbag_to_csv.py` (make sure roscore running)

