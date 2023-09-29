cmake_minimum_required(VERSION 2.8.3)
project(unbiased_aic)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)


#Add to message files
add_message_files(
      FILES
      reference.msg
    )

#Add to message generator
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


###################################
## catkin specific configuration ##
###################################

catkin_package( CATKIN_DEPENDS
	roscpp
	rospy
	sensor_msgs
	std_msgs
  geometry_msgs
  message_generation
	message_runtime
)

###########
## Build ##
###########

# Add AIC library
include_directories(
  include
	${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/classes/uAIC.cpp
  src/classes/PD.cpp
)


## Declare a C++ executable

add_executable(${PROJECT_NAME}_uAIC src/uAIC_controller.cpp)
add_dependencies(${PROJECT_NAME}_uAIC ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_uAIC ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_PD src/PD_controller.cpp)
add_dependencies(${PROJECT_NAME}_PD ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_PD ${catkin_LIBRARIES} ${PROJECT_NAME})