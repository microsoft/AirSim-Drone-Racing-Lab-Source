cmake_minimum_required(VERSION 3.5.0)
project(AirLib)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

# moveOnSpline deps
find_package (gflags HINTS ${AIRSIM_ROOT}/external/gflags_airsim/build REQUIRED)
set(glog_INCLUDE_DIR ${AIRSIM_ROOT}/AirLib/deps/gloglib/include)
set(nlopt_DIR ${AIRSIM_ROOT}/AirLib/deps/nloptlib/)
set(nlopt_INCLUDE_DIR ${nlopt_DIR}/include)

IF (UNIX)
	set(glog_LIBRARIES ${AIRSIM_ROOT}/AirLib/deps/gloglib/lib/libglog.a)
	set (nlopt_LIBRARIES ${AIRSIM_ROOT}/external/nlopt_airsim/build/libnlopt.a)
ELSE ()
	set(glog_LIBRARIES ${AIRSIM_ROOT}/AirLib/deps/gloglib/lib/${BUILD_PLATFORM}/${CMAKE_BUILD_TYPE}/glog.lib)
	set (nlopt_LIBRARIES ${nlopt_DIR}/${BUILD_PLATFORM}/${CMAKE_BUILD_TYPE}/nlopt.lib)
ENDIF ()

IncludeEigen()

include_directories(
  ${AIRSIM_ROOT}/AirLib/
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${RPC_LIB_INCLUDES}
  # moveOnSpline deps
  ${gflags_INCLUDE_DIR} 
  ${glog_INCLUDE_DIR}
  ${nlopt_DIR}/util 
  ${nlopt_DIR}/api
  ${nlopt_INCLUDE_DIR}
)

file(GLOB_RECURSE ${PROJECT_NAME}_sources 
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/common/common_utils/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/safety/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/car/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/api/*.cpp
  # moveOnSpline source files
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/mav_trajectory_generation/motion_defines.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/mav_trajectory_generation/polynomial.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/mav_trajectory_generation/rpoly/rpoly_ak1.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/mav_trajectory_generation/segment.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/mav_trajectory_generation/timing.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/mav_trajectory_generation/trajectory.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/mav_trajectory_generation/trajectory_sampling.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/mav_trajectory_generation/vertex.cpp
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_sources})

CommonTargetLink()
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
target_link_libraries(${PROJECT_NAME} MavLinkCom)

# moveOnSpline deps
target_link_libraries(${PROJECT_NAME} ${gflags_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${glog_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${nlopt_LIBRARIES})

#string(SUBSTRING ${CMAKE_STATIC_LINKER_FLAGS} 9 -1 "BUILD_PLATFORM")
#find_package(Threads REQUIRED)
#target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
#target_link_libraries(${PROJECT_NAME} AirLib)
#target_link_libraries(${PROJECT_NAME} MavLinkCom)
#target_link_libraries(${PROJECT_NAME} ../AirLib/deps/rpclib/lib/${BUILD_PLATFORM}/$(Configuration)/rpc)
