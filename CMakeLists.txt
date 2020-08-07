cmake_minimum_required(VERSION 2.8.3)
project(ros_ground_station)

# locate dependencies
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  vision_msgs
  std_msgs
  cv_bridge
  message_filters
)

find_package(OpenCV)

find_package(Boost REQUIRED)

# define catkin package
catkin_package(
  CATKIN_DEPENDS nodelet roscpp image_transport sensor_msgs vision_msgs message_filters
)

# include paths
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${Boost_INCLUDE_DIRS})

# enable c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# sender, receiver nodes, and also linking powprof library explicitly here
add_executable(sender src/node_sender.cpp)
target_link_libraries(sender ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_INCLUDE_DIRS} powprof)
add_executable(receiver src/node_receiver.cpp)
target_link_libraries(receiver ${catkin_LIBRARIES} powprof)
add_executable(planner src/node_planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} powprof)
add_executable(saver src/node_saver.cpp)
target_link_libraries(saver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_INCLUDE_DIRS} powprof)
add_executable(powprofros src/node_powprof.cpp)
target_link_libraries(powprofros ${catkin_LIBRARIES} powprof)