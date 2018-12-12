cmake_minimum_required(VERSION 3.4)
project(RoboLib)

FIND_PACKAGE(Qt5Widgets)
FIND_PACKAGE(Qt5Gui)
FIND_PACKAGE(OpenSceneGraph REQUIRED COMPONENTS
    osgDB osgGA osgUtil osgViewer osgText)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

INCLUDE_DIRECTORIES(
  ${OPENSCENEGRAPH_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

IF (NOT WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra")
    SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
ENDIF()

IF (NOT TARGET gtest)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  add_subdirectory(lib/googletest)
ENDIF()

include_directories(${CERES_INCLUDE_DIRS})

add_library(RoboLib STATIC
  robot.h
  robot.cpp
  laserscanner.h
  laserscanner.cpp
  robotmapper.h
  robotmapper.cpp
  icp.h
  icp.cpp
  roboutils.h
  roboutils.cpp
  grid2d.h
  grid2d.cpp
  ceres_scan_matcher.h
  ceres_scan_matcher.cpp
  occupied_space_cost_function_2d.cpp
  slam2d.h
  slam2d.cpp
  )
target_link_libraries(RoboLib
    ${OPENSCENEGRAPH_LIBRARIES}	
    Qt5::Widgets
    Qt5::Gui
    ${CERES_LIBRARIES}
)

add_executable(TestRoboLib
  test_robot.cpp
  test_robotmapper.cpp
  test_icp.cpp
  test_roboutils.cpp
  test_grid2d.cpp
  test_ceres_scan_matcher.cpp
  test_slam2d.cpp
  )
target_link_libraries(TestRoboLib gtest gtest_main RoboLib ${CERES_LIBRARIES})
