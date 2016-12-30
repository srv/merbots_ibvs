cmake_minimum_required(VERSION 2.8.3)

project(merbots_ibvs)

# Finding headers and launch files for QtCreator
file(GLOB_RECURSE HDRS ${CMAKE_CURRENT_SOURCE_DIR} *.h *.hpp)
file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR} *.cxx *.cpp *.c *.py)
file(GLOB_RECURSE LNCH ${CMAKE_CURRENT_SOURCE_DIR} *.launch)

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs image_transport cv_bridge dynamic_reconfigure tf auv_msgs std_srvs)

# System dependencies
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Dynamic reconfigure configuration
generate_dynamic_reconfigure_options(
        cfg/IBVS.cfg
)

# Defining the package
catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs image_transport cv_bridge dynamic_reconfigure tf auv_msgs std_srvs
	DEPENDS OpenCV
)

###########
## Build ##
###########

# Including directories.
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Custom target to include headers and launch files in QtCreator
add_custom_target(dtarget_merbots_ibvs SOURCES ${HDRS} ${SRCS} ${LNCH})

# Image-based Visual Servoing
add_executable(ibvs
    src/nodes/ibvs.cpp
)
add_dependencies(ibvs ${PROJECT_NAME}_gencfg)
target_link_libraries(ibvs ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
