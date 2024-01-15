cmake_minimum_required(VERSION 3.0.2)
project(scale_truck_control_gui)

find_package(catkin REQUIRED std_msgs roscpp roslib)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(OpenCV 4.4.0 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

add_executable(gui_node
  src/controller.ui
  src/main.cpp
  src/controller.cpp
  src/qTh.cpp
  src/sock_udp.cpp
  include/controller.h
  include/qTh.h
)

target_link_libraries(gui_node
  ${catkin_LIBRARIES}
  Qt5::Widgets
  ${OpenCV_LIBS}
  Threads::Threads
)

