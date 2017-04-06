cmake_minimum_required(VERSION 2.8.3)
project(harris)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
)

find_package(OpenCV)

#################################
## Add Dynamic Reconfigure API ##
#################################
#generate_dynamic_reconfigure_options(
#  cfg/harrisFeatures.cfg
#)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imagepub
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport roscpp
)


###########
## Build ##
###########

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## add the C++ harris_corners 
add_executable(harris_corners src/harris_corners.cpp)
add_dependencies(harris_corners 
	${catkin_EXPORTED_TARGETS} 
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(harris_corners
	${catkin_LIBRARIES} 
	${OpenCV_LIBRARIES}
)