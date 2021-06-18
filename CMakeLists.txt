cmake_minimum_required(VERSION 2.8.3)
project(battery_mockup)

find_package(catkin REQUIRED COMPONENTS genmsg rospy std_msgs message_generation)

add_service_files(
FILES
CheckPercentage.srv
)
catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)
