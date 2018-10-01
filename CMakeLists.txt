cmake_minimum_required(VERSION 2.8.3)
project(rospix_utils)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  std_msgs
  dynamic_reconfigure
  message_generation
  rospix
  cv_bridge
)

# add_message_files(FILES
#   message.msg
# )

# generate_messages(DEPENDENCIES
#  std_msgs
#  geometry_msgs
# )

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs cv_bridge
)

set(CMAKE_CXX_FLAGS "-std=c++11")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# ascii art image printer
add_executable(ascii src/ascii.cpp)
add_dependencies(ascii ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ascii
  ${catkin_LIBRARIES}
)

# image normalizing republisher
add_executable(normalizer src/normalizer.cpp)
add_dependencies(normalizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(normalizer
  ${catkin_LIBRARIES}
)

# image ascii saver
add_executable(saver src/saver.cpp)
add_dependencies(saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(saver
  ${catkin_LIBRARIES}
)

# image batch exposure controller
add_executable(controller src/controller.cpp)
add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller
  ${catkin_LIBRARIES}
)
