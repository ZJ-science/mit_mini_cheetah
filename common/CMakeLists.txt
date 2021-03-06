# common library (libbiomimetics)

cmake_minimum_required(VERSION 2.8.2)

project(libbiomimetics)

# Pull in Google Test
include(CTest)
if (CMAKE_VERSION VERSION_LESS 3.2)
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
else()
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
endif()
include(DownloadProject.cmake)
download_project(PROJ                googletest
                 GIT_REPOSITORY      https://github.com/google/googletest.git
                 GIT_TAG             master
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
message(${googletest_SOURCE_DIR})
message(${googletest_BINARY_DIR})
if (CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories("${gtest_SOURCE_DIR}/include"
                        "${gmock_SOURCE_DIR}/include")
endif()

# Source/Include files
include_directories(".")
include_directories("include/" )                 # common includes
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("../lcm-types/cpp/")
include_directories("../third-party/inih")       # inih includes
include_directories("../third-party/osqp/include")       # osqp includes
include_directories("../third-party/ParamHandler")
include_directories("../robot/include/")
include_directories("../robot/include/rt")
include_directories(${CMAKE_BINARY_DIR})

# Casadi include
include_directories(SYSTEM "../third-party/casadi/")
include_directories(${CMAKE_BINARY_DIR}/third-party/casadi/)

file(GLOB_RECURSE sources "src/*.cpp" "../robot/include/rt/rt_interface_lcm.h" "../robot/include/rt/rt_sbus.h" "../robot/src/rt/rt_interface_lcm.cpp" "../robot/src/rt/rt_sbus.cpp" "../robot/include/rt/rt_serial.h" "../robot/src/rt/rt_serial.cpp")                   # common library cpp files
file(GLOB_RECURSE test_sources "test/test_*.cpp")             # test cpp files

# Library
add_library(biomimetics SHARED ${sources})       # produce a library used by sim/robot
target_link_libraries(biomimetics inih dynacore_param_handler )


# Test
add_executable(test-common ${test_sources} ${sources})
target_link_libraries(test-common robot)
target_link_libraries(test-common gtest gmock_main lcm rt inih osqp dynacore_param_handler pthread)
target_link_libraries(test-common Goldfarb_Optimizer)
target_link_libraries(test-common WBC_state)
target_link_libraries(test-common JCQP)
target_link_libraries(test-common casadi)



add_test(NAME example_test COMMAND test-common)

