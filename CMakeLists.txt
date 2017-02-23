cmake_minimum_required(VERSION 2.8.3)
project(asr_robot_model_services)

add_definitions(-std=gnu++0x -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  message_generation
  sensor_msgs
  nav_msgs
  asr_msgs
  actionlib
  tf
  eigen_conversions
  tf_conversions
  std_srvs
  visualization_msgs
  urdf
  kdl_parser
  costmap_2d
  dynamic_reconfigure
)


find_package(Boost REQUIRED COMPONENTS system thread)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(rsc/DynamicParameters.cfg)

add_message_files(
        DIRECTORY
        msg
        FILES
        RobotStateMessage.msg
)


## Generate services in the 'srv' folder
add_service_files(
   DIRECTORY
   srv
   FILES
   CalculateCameraPose.srv
   CalculateCameraPoseCorrection.srv
   IsPositionAllowed.srv
   GetPose.srv
   GetDistance.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs  # Or other packages containing msgs
        geometry_msgs
        sensor_msgs
        asr_msgs
        visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib message_runtime costmap_2d
  DEPENDS system_lib
)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp library
add_library(
        ${PROJECT_NAME}_packages
        src/rating/IKRatingModule.cpp
        src/robot_model/RobotModel.cpp
        src/robot_model/RobotState.cpp
	src/helper/MathHelper.cpp
        src/helper/TypeHelper.cpp
        src/helper/DebugHelper.cpp
	include/robot_model_services/typedef.hpp
)

add_library(
        ${PROJECT_NAME}_packages_impl
        src/rating/impl/NavigationPathIKRatingModule.cpp
        src/rating/impl/SimpleIKRatingModule.cpp
        src/rating/impl/AngleApproximationIKRatingModule.cpp
        src/robot_model/impl/MILDRobotModel.cpp
        src/robot_model/impl/MILDRobotModelWithExactIK.cpp
        src/robot_model/impl/MILDRobotModelWithApproximatedIK.cpp
        src/robot_model/impl/MILDRobotState.cpp
        include/robot_model_services/robot_model/RobotModelAbstractFactory.hpp
        include/robot_model_services/robot_model/impl/MILDRobotModelWithApproximatedIKFactory.hpp
        include/robot_model_services/robot_model/impl/MILDRobotModelWithExactIKFactory.hpp
)



target_link_libraries(
        ${PROJECT_NAME}_packages
        ${catkin_LIBRARIES}
)

target_link_libraries(
        ${PROJECT_NAME}_packages_impl
        ${PROJECT_NAME}_packages
	glpk
        ${catkin_LIBRARIES}
)


add_executable(
        ${PROJECT_NAME}
        src/RobotModel_Service.cpp
)

add_dependencies(
        ${PROJECT_NAME}
        ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(
        ${PROJECT_NAME}
        ${PROJECT_NAME}_packages_impl
        glpk
        ${catkin_LIBRARIES}
)

#TEST CASES
#SET VARIABLES TO ENABLE/DISABLE TEST BUILDING
set(BUILD_ALL_TESTS FALSE)

set (BUILD_IK_TEST ${BUILD_ALL_TESTS})

set (TEST_LINK_LIBRARIES 
        ${PROJECT_NAME}_packages
        ${PROJECT_NAME}_packages_impl
        glpk
        ${catkin_LIBRARIES}
)

if (BUILD_IK_TEST)
MESSAGE ("Building Tests:")
add_library(
        ${PROJECT_NAME}_test_core
        src/test_cases/BaseTest.cpp
)

target_link_libraries(
	${PROJECT_NAME}_test_core
        ${TEST_LINK_LIBRARIES}
)
endif ()

if (BUILD_IK_TEST)
MESSAGE (" - IK Test")
add_executable(
        ${PROJECT_NAME}_ik_test
        src/test_cases/IKTest.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_ik_test
        ${PROJECT_NAME}_test_core
        ${TEST_LINK_LIBRARIES}
)
endif ()




