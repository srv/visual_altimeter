cmake_minimum_required(VERSION 2.8.3)
project(visual_altimeter)
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	pcl_ros
	std_msgs
	sensor_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(visual_altimeter src/visual_altimeter.cpp)
target_link_libraries(visual_altimeter ${catkin_LIBRARIES})