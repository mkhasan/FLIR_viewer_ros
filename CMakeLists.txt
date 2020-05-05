cmake_minimum_required(VERSION 2.8.3)
project(FLIR_viewer)


add_compile_options(-std=c++14)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ggdb -Wall -D_GNU_SOURCE=1 -D_REENTRANT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -Wall -D_GNU_SOURCE=1 -D_REENTRANT")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cam_viewer
  
)


include_directories(
  include
  /usr/local/include
  /usr/include/spinnaker
  ${catkin_INCLUDE_DIRS}
)

find_library( SPINNAKER_LIB Spinnaker)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp src/${PROJECT_NAME}.cpp )

catkin_package(
   INCLUDE_DIRS include 
   LIBRARIES ${PROJECT_NAME} 
   CATKIN_DEPENDS roscpp rospy std_msgs 
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${SPINNAKER_LIB})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp )
 
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})


