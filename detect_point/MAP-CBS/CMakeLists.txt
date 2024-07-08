cmake_minimum_required(VERSION 3.0)

project(CBS_DETECT_POINT)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Debug)

set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/cbs_common/func_common.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/cbs_common/dubins.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/cbs_processing/CBS.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/cbs_processing/CBSDataStructures.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/cbs_processing/LowLevel.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cbs_common
                    ${CMAKE_CURRENT_SOURCE_DIR}/cbs_processing)

add_executable(main_cmake main.cpp ${SRC})

