cmake_minimum_required(VERSION 3.4)
project(FinalProject)

FIND_PACKAGE(Qt5Widgets)
FIND_PACKAGE(Qt5Gui)
FIND_PACKAGE(OpenSceneGraph REQUIRED COMPONENTS
    osgDB osgGA osgUtil osgViewer osgText)
find_package(Eigen3 REQUIRED)

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

add_library(RoboLib STATIC
  robot.h
  robot.cpp
  laserscanner.h
  laserscanner.cpp
  )
target_link_libraries(RoboLib
    ${OPENSCENEGRAPH_LIBRARIES}	
)

SET(MYSOURCE
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindowform.ui
    osgwidget.h
    osgwidget.cpp
    robotupdatecallback.cpp
    models.cpp
    laserscanwidget.cpp
    )

add_executable(${PROJECT_NAME}
    ${MYSOURCE}
    )

target_link_libraries(${PROJECT_NAME}
    ${OPENSCENEGRAPH_LIBRARIES}	
    Qt5::Widgets
    Qt5::Gui
    RoboLib
)

add_executable(TestRobot
  test_robot.cpp
  )
target_link_libraries(TestRobot gtest gtest_main RoboLib)
