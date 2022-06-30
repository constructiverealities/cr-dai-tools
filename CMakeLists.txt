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
                GIT_TAG 1281dc90cb35b3d67cf45a0233d6352194d216eb
    )

    FetchContent_MakeAvailable(depthai-core)
    project(cr-dai-tools)
    find_package(depthai REQUIRED)
else()
    project(cr-dai-tools)
    find_package(depthai REQUIRED)
endif()

set(CMAKE_CXX_STANDARD 17)

if(NOT ROS_BUILD)
    add_subdirectory(dai-tools)
endif()
add_subdirectory(ros)

if(NOT ROS_BUILD)
    add_executable(cr-dai-example ./apps/example.cc)
    target_link_libraries(cr-dai-example cr-dai-tools)

    install(TARGETS cr-dai-example DESTINATION bin)
    if(WIN32)
        install(FILES $<TARGET_PDB_FILE:cr-dai-example> DESTINATION bin OPTIONAL)
    endif()
endif()
