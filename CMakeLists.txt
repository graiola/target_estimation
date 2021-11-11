cmake_minimum_required(VERSION 2.8.3)
project(target_estimation)

#find_package(Eigen3 REQUIRED)
find_package(catkin QUIET COMPONENTS roscpp rt_logger)
find_package(PkgConfig)
pkg_check_modules(EIGEN eigen3)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

if (${roscpp_FOUND} AND ${rt_logger_FOUND})
    add_definitions(-DLOGGER_ON)
    message(STATUS "${Yellow}ROS and RT_LOGGER found, add logger${ColorReset}")
endif()

# C++ 11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories(
 include
 ${EIGEN_INCLUDE_DIRS}
 ${YAMLCPP_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
)

add_library(kalman SHARED
  src/kalman.cpp
)

add_library(target SHARED
  src/target_interface.cpp
  src/types/angular_rates.cpp
  src/types/angular_velocities.cpp
  src/types/projectile.cpp
  src/types/uniformly_accelerated.cpp
  src/target_manager.cpp
  src/solver.cpp
)
target_link_libraries(target kalman yaml-cpp ${catkin_LIBRARIES})

add_library(target_c SHARED
  src/target_manager_c.cpp
)
target_link_libraries(target_c target kalman)

install(TARGETS target target_c kalman
        ARCHIVE DESTINATION include
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include
)

if (${roscpp_FOUND})
    message(STATUS "${Yellow}ROS found, compiling ros dependent code${ColorReset}")
    find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs message_generation tf)

    add_message_files(FILES TargetEstimation.msg)

    generate_messages(
        DEPENDENCIES
        geometry_msgs)

    catkin_package(
        LIBRARIES target kalman
        INCLUDE_DIRS include
    )

    include_directories(include ${YAMLCPP_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${rt_logger_INCLUDE_DIRS})

    add_library(target_ros src/target_manager_ros.cpp)
    target_link_libraries(target_ros target ${catkin_LIBRARIES} ${rt_logger_LIBRARIES})

    add_executable(target_node src/target_node.cpp)
    target_link_libraries(target_node target_ros)

    add_executable(target_manager_test test/target_manager_test.cpp)
    target_link_libraries(target_manager_test target ${catkin_LIBRARIES} ${rt_logger_LIBRARIES} gtest)

    install(TARGETS target target_c target_ros kalman target_node
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(DIRECTORY launch/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
      PATTERN ".svn" EXCLUDE)

    install(DIRECTORY models/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
      PATTERN ".svn" EXCLUDE)

   install(DIRECTORY rviz/
     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
     PATTERN ".svn" EXCLUDE)

   install(DIRECTORY plot_juggler_files/
     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plot_juggler_files
     PATTERN ".svn" EXCLUDE)
endif()

