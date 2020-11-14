cmake_minimum_required(VERSION 2.8.3)
project(teknic_motor_controller)

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)
# find_package(CMAKE_DL_LIBS REQUIRED)
# add_compile_options(-pthread)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  # std_msgs
  # message_generation
  # sensor_msgs
)

include_directories(
  include
  include/inc
  include/inc/inc-private
  include/inc/inc-private/linux
  include/inc/inc-private/sFound
  include/inc/inc-pub
  include/LibINI/inc
  include/LibLinuxOS/inc
  include/LibXML/inc
  ${catkin_INCLUDE_DIRS}
  ${Threads_INCLUDE_DIRS}
)

catkin_package(
  # INCLUDE_DIRS include include/inc include/inc/inc-private include/inc/inc-private/linux include/inc/inc-private/sFound include/inc/inc-pub include/LibINI/inc include/LibLinuxOS/inc include/LibXML/inc 
  CATKIN_DEPENDS roscpp # message_runtime std_msgs # sensor_msgs
)

add_executable(HelloWorld
  src/SDK_Examples/HelloWorld/HelloWorld.cpp 
)

add_library(LibINI
  src/LibINI/src/dictionary.cpp
  src/LibINI/src/iniparser.cpp
)

add_library(LibLinuxOS
  src/LibLinuxOS/src/tekEventsLinux.cpp
  src/LibLinuxOS/src/tekThreadsLinux.cpp

  src/LibLinuxOS/src/version.cpp
)

add_library(LibXML
  src/LibXML/src/ErrCodeStr.cpp
  src/LibXML/src/pugixml.cpp
)

add_library(sFoundation
  src/sFoundation/src/converterLib.cpp
  src/sFoundation/src/cpmAPI.cpp
  src/sFoundation/src/cpmClassImpl.cpp
  src/sFoundation/src/iscAPI.cpp
  src/sFoundation/src/lnkAccessCommon.cpp
  src/sFoundation/src/meridianNet.cpp
  src/sFoundation/src/netCmdAPI.cpp
  src/sFoundation/src/netCoreFmt.cpp
  src/sFoundation/src/SerialEx.cpp
  src/sFoundation/src/sysClassImpl.cpp
  src/sFoundation/src-linux/lnkAccessLinux.cpp
  src/sFoundation/src-linux/SerialLinux.cpp
  )
message(${CMAKE_THREAD_LIBS_INIT})

target_link_libraries(sFoundation LibINI LibLinuxOS LibXML ${CMAKE_DL_LIBS})
target_link_libraries(HelloWorld sFoundation ${CMAKE_THREAD_LIBS_INIT})

