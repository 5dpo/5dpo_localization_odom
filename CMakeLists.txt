cmake_minimum_required(VERSION 3.5)
project(sdpo_localization_odom)



# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



###################################################
#####  >>> ROBOT OPERATING SYSTEM (ROS) 2 <<< #####
###################################################

if("$ENV{ROS_VERSION}" STREQUAL "2")

  # Dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sdpo_drivers_interfaces REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)



  # Node Executable
  add_executable(${PROJECT_NAME}_wh
    src/${PROJECT_NAME}/OdomWhMotorDrive.cpp
    src/${PROJECT_NAME}/OdomWhDiff.cpp
    src/${PROJECT_NAME}/OdomWhOmni3.cpp
    src/${PROJECT_NAME}/OdomWhOmni4.cpp
    src/${PROJECT_NAME}/OdomWhROS2.cpp
    src/${PROJECT_NAME}/utils.cpp
    src/main_ros2_odom_wh.cpp
  )

  target_include_directories(${PROJECT_NAME}_wh PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  
  ament_target_dependencies(${PROJECT_NAME}_wh
    rclcpp
    geometry_msgs
    nav_msgs
    sdpo_drivers_interfaces
    tf2
    tf2_ros
  )



  # Node Executable Installation Rules
  install(
    TARGETS ${PROJECT_NAME}_wh
    DESTINATION lib/${PROJECT_NAME}
  )
  
  
  
  # Launch Files Installation Rules
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )



  # Default Config Installation Rules
  # install(
  #   DIRECTORY config
  #   DESTINATION share/${PROJECT_NAME}
  # )



  # Testing
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()



###################################################
#####  >>> ROBOT OPERATING SYSTEM (ROS) 1 <<< #####
###################################################

elseif("$ENV{ROS_VERSION}" STREQUAL "1")


##########################
#####  !!! ERROR !!! #####
##########################

else()

  message(FATAL_ERROR "No valid version of ROS is installed or environment variables not set")

endif()
