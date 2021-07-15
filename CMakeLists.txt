cmake_minimum_required(VERSION 2.8.3)
project(battery_mockup)

find_package(catkin REQUIRED COMPONENTS genmsg rospy std_msgs actionlib_msgs actionlib message_generation geometry_msgs)

add_action_files(DIRECTORY action FILES CheckBattery.action Charge.action)

add_service_files(
FILES
CheckPercentage.srv
)
catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

roslaunch_add_file_check(launch/launch_battery.launch)
