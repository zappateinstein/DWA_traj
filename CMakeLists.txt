PROJECT(DWA_traj)
cmake_minimum_required(VERSION 2.8)
include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)

if("${CMAKE_SYSTEM_PROCESSOR_DEFAULTTUNE}" MATCHES "core2-64")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/simulator)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ugv)

else()
    if(DEFINED CMAKE_SYSTEM_PROCESSOR_DEFAULTTUNE)
        warn("${PROJECT_NAME} will not be built for ${CMAKE_SYSTEM_PROCESSOR_DEFAULTTUNE} architecture")
    else()
        warn("${PROJECT_NAME} will not be built for ${CMAKE_SYSTEM_PROCESSOR} architecture")
    endif()
endif()
