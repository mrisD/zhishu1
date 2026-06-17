# guohua_ws  


###启动mtc抓取程序  
ros2 launch zhishu_robot_mtc_pick_place mtc_demos.launch.py  


ros2 launch zhishu_robot_hardware driver.launch.py slave_position:=2

ros2 topic pub --once /drive3/relative_deg std_msgs/msg/Float64 "{data: 90.0}"