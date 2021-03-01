cmake_minimum_required(VERSION 3.1.2)
project(rospix_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  std_msgs
  dynamic_reconfigure
  cv_bridge
  rospy
  mrs_lib
  rad_msgs
  nodelet
)

generate_dynamic_reconfigure_options(
  config/integrator.cfg
  )

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs cv_bridge rospy mrs_lib rad_msgs nodelet dynamic_reconfigure
  LIBRARIES Integrator Normalizer Dosimeter
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

# ascii art image printer
add_executable(ascii src/ascii.cpp)
add_dependencies(ascii ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ascii
  ${catkin_LIBRARIES}
)

# image normalizing republisher
add_library(Normalizer src/normalizer.cpp)

add_dependencies(Normalizer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(Normalizer
  ${catkin_LIBRARIES}
)

# image normalizing republisher
add_library(Integrator src/integrator.cpp)

add_dependencies(Integrator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(Integrator
  ${catkin_LIBRARIES}
)

# image normalizing republisher
add_library(Dosimeter src/dosimeter.cpp)

add_dependencies(Dosimeter
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(Dosimeter
  ${catkin_LIBRARIES}
)

# image normalizing republisher
add_executable(masker src/masker.cpp)
add_dependencies(masker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(masker
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
