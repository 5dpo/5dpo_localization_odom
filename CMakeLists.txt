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

  message(AUTHOR_WARNING "Project version in ROS 1 may be deprecated")

  # Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    nav_msgs
    sdpo_drivers_interfaces
    tf2
    tf2_ros
  )

  # Catkin specific configuration
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_wh
    CATKIN_DEPENDS
      roscpp
      geometry_msgs
      nav_msgs
      sdpo_drivers_interfaces
      tf2
      tf2_ros
    # DEPENDS system_lib
  )

  # Build
  # - additional header files
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  # - declare a C++ library
  # add_executable(${PROJECT_NAME}_wh
  #   src/${PROJECT_NAME}/OdomWhMotorDrive.cpp
  #   src/${PROJECT_NAME}/OdomWhDiff.cpp
  #   src/${PROJECT_NAME}/OdomWhOmni3.cpp
  #   src/${PROJECT_NAME}/OdomWhOmni4.cpp
  #   src/${PROJECT_NAME}/OdomWhROS1.cpp
  #   src/${PROJECT_NAME}/utils.cpp
  # )

  # - declare a C++ executable
  add_executable(${PROJECT_NAME}_wh
    src/${PROJECT_NAME}/OdomWhMotorDrive.cpp
    src/${PROJECT_NAME}/OdomWhDiff.cpp
    src/${PROJECT_NAME}/OdomWhOmni3.cpp
    src/${PROJECT_NAME}/OdomWhOmni4.cpp
    src/${PROJECT_NAME}/OdomWhROS1.cpp
    src/${PROJECT_NAME}/utils.cpp
    src/main_ros1_odom_wh.cpp
  )

  # - target dependencies
  add_dependencies(${PROJECT_NAME}_wh
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  # add_dependencies(${PROJECT_NAME}      # for the library
  #   ${${PROJECT_NAME}_EXPORTED_TARGETS}
  #   ${catkin_EXPORTED_TARGETS}
  # )

  # - specify linking libraries
  target_link_libraries(${PROJECT_NAME}_wh
    ${catkin_LIBRARIES}
  )

  # - installation rules
  install(TARGETS ${PROJECT_NAME}_wh
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  # install(TARGETS ${PROJECT_NAME}       # for the library
  #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  # )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )

  install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )



##########################
#####  !!! ERROR !!! #####
##########################

else()

  message(FATAL_ERROR "No valid version of ROS is installed or environment variables not set")

endif()
