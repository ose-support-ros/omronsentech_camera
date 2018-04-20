cmake_minimum_required(VERSION 2.8.3)
project(omronsentech_camera)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  image_transport
  message_generation
  roscpp
  roslaunch
  roslint
  rospy
  sensor_msgs
  std_msgs
  nodelet
  pluginlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   GenTLInfo.msg
   DeviceConnection.msg
   GenICamEvent.msg
   ElementFloat64.msg
   ElementInt64.msg
   ElementBool.msg
   ElementString.msg
   Event.msg
   Chunk.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    GetDeviceList.srv
    GetModuleList.srv
    GetSDKInfo.srv
    GetGigEIP.srv
    SetGigEIP.srv
    GetGigEIPi.srv
    SetGigEIPi.srv
    ReadNode.srv
    ReadNodeBool.srv
    ReadNodeEnum.srv
    ReadNodeInt.srv
    ReadNodeFloat.srv
    ReadNodeString.srv
    WriteNode.srv
    WriteNodeBool.srv
    WriteNodeEnumInt.srv
    WriteNodeEnumStr.srv
    WriteNodeInt.srv
    WriteNodeFloat.srv
    WriteNodeString.srv
    EnableChunk.srv
    EnableTrigger.srv
    EnableEventNode.srv
    EnableImageAcquisition.srv
    EnableEventAcquisition.srv
    GetImageAcquisitionStatus.srv
    GetEventAcquisitionStatusList.srv
    GetEventNodeStatusList.srv
    GetChunkList.srv
    GetTriggerList.srv
    GetEnumList.srv
    GetGenICamNodeInfo.srv
    SendSoftTrigger.srv
    ExecuteNode.srv
    GetLastError.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    sensor_msgs 
    std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
#generate_dynamic_reconfigure_options(
#    cfg/StCamera.cfg 
#)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES omronsentech_camera stcamera_node stcamera_nodelet
  CATKIN_DEPENDS nodelet camera_info_manager image_transport roscpp roslaunch roslint rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

roslint_cpp(
    src/${PROJECT_NAME}/main.cpp
    src/${PROJECT_NAME}/stparameter.cpp
    src/${PROJECT_NAME}/stcamera_node.cpp
    src/${PROJECT_NAME}/stcamera_interface.cpp
    src/${PROJECT_NAME}/stcamera_interface_u3v.cpp
    src/${PROJECT_NAME}/stcamera_interface_gev.cpp
    include/${PROJECT_NAME}/stparameter.h
    include/${PROJECT_NAME}/stcamera_node.h
    include/${PROJECT_NAME}/stcamera_interface.h
    include/${PROJECT_NAME}/stcamera_interface_u3v.h
    include/${PROJECT_NAME}/stcamera_interface_gev.h
)
###########
## Build ##
###########
set (CMAKE_CXX_STANDARD 11)

# Custom target to install third party library
EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)

message (STATUS "${ENV}")
message (STATUS "$ENV{STAPI_ROOT_PATH}")
if (DEFINED ENV{STAPI_ROOT_PATH})
  message (STATUS "STAPI_ROOT_PATH setting is detected.")
else()
  message (FATAL_ERROR "\nPlease install SentechSDK (can be downloaded from www.sentech.co.jp), load the SDK environment settings\n (e.g. $ source /opt/sentech/.stprofile) and try again.")

endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  $ENV{STAPI_ROOT_PATH}/include/GenICam
  $ENV{STAPI_ROOT_PATH}/include/StApi
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

link_directories($ENV{STAPI_ROOT_PATH}/lib)
link_directories($ENV{STAPI_ROOT_PATH}/lib/GenICam)

set (SOURCE_FILES
    src/${PROJECT_NAME}/stcamera_node.cpp
    src/${PROJECT_NAME}/stparameter.cpp
    src/${PROJECT_NAME}/stcamera_interface.cpp
    src/${PROJECT_NAME}/stcamera_interface_u3v.cpp
    src/${PROJECT_NAME}/stcamera_interface_gev.cpp
)
## Declare a C++ library
add_library(${PROJECT_NAME}
  ${SOURCE_FILES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/omronsentech_camera_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    libGCBase.so libGenApi.so libStApi_TL.so -lpthread
)

add_library(stcamera_nodelet
  src/${PROJECT_NAME}/stcamera_nodelet.cpp
  ${SOURCE_FILES}
)

target_link_libraries(stcamera_nodelet
    ${catkin_LIBRARIES}
    libGCBase.so libGenApi.so libStApi_TL.so -lpthread
)

add_executable(
    stcamera_node
    src/${PROJECT_NAME}/main.cpp
)

target_link_libraries(stcamera_node
    ${PROJECT_NAME}
    libGCBase.so libGenApi.so libStApi_TL.so -lpthread
)

add_executable(grabber examples/cpp/grabber.cpp)

target_link_libraries(grabber
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  libGCBase.so libGenApi.so libStApi_TL.so -lpthread
)

add_dependencies(stcamera_nodelet 
  ${catkin_EXPORTED_LIBRARIES}
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
add_dependencies(stcamera_node 
  ${catkin_EXPORTED_LIBRARIES}
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

#add_dependencies(stcamera_node ${PROJECT_NAME}_gencfg)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} stcamera_node stcamera_nodelet
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}-test test/unittest_omronsentech.cpp)
#target_link_libraries(${PROJECT_NAME}-test 
#  ${catkin_LIBRARIES}
#  ${PROJECT_NAME}
#  libGCBase.so libGenApi.so libStApi_TL.so -lpthread
#)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
