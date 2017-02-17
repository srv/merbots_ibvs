cmake_minimum_required(VERSION 2.8.3)

project(merbots_ibvs)

# Finding headers and launch files for QtCreator
file(GLOB_RECURSE HDRS ${CMAKE_CURRENT_SOURCE_DIR} *.h *.hpp)
file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR} *.cxx *.cpp *.c *.py)
file(GLOB_RECURSE LNCH ${CMAKE_CURRENT_SOURCE_DIR} *.launch)

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs image_transport cv_bridge dynamic_reconfigure tf auv_msgs std_srvs nodelet message_generation merbots_tracking)

# System dependencies
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Message files
add_message_files(
    FILES
    IBVSInfo.msg
)

# Generating messages
generate_messages(
    DEPENDENCIES
    std_msgs
)

# Dynamic reconfigure configuration
generate_dynamic_reconfigure_options(
        cfg/IBVS.cfg
)

# Defining the package
catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs image_transport cv_bridge dynamic_reconfigure tf auv_msgs std_srvs nodelet message_runtime merbots_tracking
    DEPENDS OpenCV
)

###########
## Build ##
###########

# Including directories.
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Custom target to include headers and launch files in QtCreator
add_custom_target(dtarget_merbots_ibvs SOURCES ${HDRS} ${SRCS} ${LNCH})

# Image-based Visual Servoing
add_library(ibvs
  src/pid/PID.cpp
  src/ibvs/IBVS.cpp)
add_dependencies(ibvs ${PROJECT_NAME}_gencfg)
target_link_libraries(ibvs ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

# IBVS Nodelet
add_library(ibvs_nodelet src/nodelets/ibvs_nodelet.cpp)
target_link_libraries(ibvs_nodelet ibvs)

# IBVS node
add_executable(ibvs_node
    src/nodes/ibvs_node.cpp
)
target_link_libraries(ibvs_node ibvs)

add_executable(target_pose_publisher_node
    src/nodes/target_pose_publisher_node.cpp
)
target_link_libraries(target_pose_publisher_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
