cmake_minimum_required(VERSION 3.0.2)
project(stream_shower)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge image_transport)
find_package(OpenCV REQUIRED )
find_package(Pangolin REQUIRED)


## Declare ROS messages and services
#add_message_files(FILES 
#                Event.msg
#		EventArray.msg
#                )
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(ROFusion_vis src/stream_vis.cpp)
target_link_libraries(ROFusion_vis ${catkin_LIBRARIES})
target_link_libraries( ROFusion_vis ${OpenCV_LIBS} )
target_link_libraries( ROFusion_vis ${Pangolin_LIBRARIES} )


#add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(ROFusion_vis ${catkin_EXPORTED_TARGETS})

