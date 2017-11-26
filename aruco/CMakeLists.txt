cmake_minimum_required(VERSION 2.8.3)
project(maruco)

#Enable C++ 11
add_compile_options(-std=c++11)

#Packages
find_package(catkin REQUIRED COMPONENTS	cv_bridge roscpp std_msgs message_generation image_transport)
find_package(OpenCV REQUIRED)

#Messages
add_message_files(FILES Marker.msg)
generate_messages(DEPENDENCIES std_msgs)

#Catkin dependencies
catkin_package(CATKIN_DEPENDS message_runtime roscpp std_msgs)

#Aruco ROS node
add_executable(maruco src/ros/ArucoNode.cpp)
add_dependencies(maruco aruco_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(maruco ${catkin_LIBRARIES} ${OpenCV_LIBS})

#Include directories
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

#Install
install(TARGETS maruco
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
