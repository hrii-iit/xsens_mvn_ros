cmake_minimum_required(VERSION 3.0.0)
project(xsens_awinda_ros)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  eigen_conversions
  roslib
  message_generation
)

# find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_message_files(FILES
  LinkState.msg
  LinkStateArray.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    sensor_msgs
    std_msgs
    tf2_ros
    eigen_conversions
    roslib

  INCLUDE_DIRS 
    include 

)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
file(GLOB ${PROJECT_NAME}_SDK_CPP_PATH src/xsens_SDK/*.cpp)
add_library(${PROJECT_NAME}_SDK SHARED ${${PROJECT_NAME}_SDK_CPP_PATH})
set_target_properties(${PROJECT_NAME}_SDK PROPERTIES LINKER_LANGUAGE CXX)

add_library(${PROJECT_NAME}_HumanDataHandler SHARED src/xsens_client/HumanDataHandler.cpp)
add_dependencies(${PROJECT_NAME}_HumanDataHandler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_HumanDataHandler ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_XSensClient SHARED src/xsens_client/XSensClient.cpp src/xsens_client/Socket.cpp)
add_dependencies(${PROJECT_NAME}_XSensClient ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_XSensClient ${catkin_LIBRARIES} ${PROJECT_NAME}_HumanDataHandler ${PROJECT_NAME}_SDK)

## Declare executables
add_executable(${PROJECT_NAME}_xsens_client_node src/xsens_client_node.cpp)
set_target_properties(${PROJECT_NAME}_xsens_client_node PROPERTIES OUTPUT_NAME "xsens_client")
add_dependencies(${PROJECT_NAME}_xsens_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_xsens_client_node ${PROJECT_NAME}_XSensClient)

add_executable(${PROJECT_NAME}_generate_human_model src/generate_human_model_node.cpp)
set_target_properties(${PROJECT_NAME}_generate_human_model PROPERTIES OUTPUT_NAME "generate_human_model")
add_dependencies(${PROJECT_NAME}_generate_human_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_generate_human_model ${PROJECT_NAME}_XSensClient)

# Install headers and libraries
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
