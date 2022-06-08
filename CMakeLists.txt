cmake_minimum_required(VERSION 3.10)

set(CMAKE_MAP_IMPORTED_CONFIG_RELWITHDEBINFO "RelWithDebInfo;Release;")

option(ROS_BUILD "Build in a ros workspace" OFF)
option(BUILD_DEPTHAI "Build all relevant dependencies" ${ROS_BUILD})

include(FetchContent)
if(BUILD_DEPTHAI)
    MESSAGE("Building depthai locally...")
    FetchContent_Declare(
            depthai-core
            GIT_REPOSITORY https://github.com/constructiverealities/depthai.git
            GIT_TAG 47c31e9596645f5ee15a4125dc21d0015c6503c6
    )

    FetchContent_MakeAvailable(depthai-core)
    project(cr-dai-tools)
else()
    project(cr-dai-tools)
    find_package(depthai REQUIRED)
endif()


set(CMAKE_CXX_STANDARD 17)
include(CheckCXXSourceCompiles)

#add_compile_options(-fsanitize=address)
#add_link_options(-fsanitize=address)
if(NOT ROS_BUILD)
    add_subdirectory(dai-tools)
endif()
add_subdirectory(ros)

if(NOT ROS_BUILD)
    add_executable(cr-dai-example ./apps/example.cc)
    target_link_libraries(cr-dai-example cr-dai-tools)

    install(TARGETS cr-dai-example DESTINATION bin)
endif()