cmake_minimum_required(VERSION 3.10)
project(dai-pipeline-tools)

set(CMAKE_CXX_STANDARD 14)
include(CheckCXXSourceCompiles)

find_package(depthai REQUIRED)

find_path(HAS_TOF_SUPPORT "ToF.hpp" PATHS ${depthai_DIR}/../../../include/depthai/pipeline/node)
if(HAS_TOF_SUPPORT)
    add_compile_options("-DDEPTHAI_HAS_TOF_NODE=1")
    message("HAS TOF SUPPORT")
else()
    message("HAS NO TOF SUPPORT")
endif()

include_directories(${depthai_INCLUDE_DIRS} include)
add_library(dai-pipeline-tools src/dai-pipeline-tools.cc src/generate_pipeline.cc)
target_link_libraries(dai-pipeline-tools depthai::core)

add_executable(dai-pipeline-example ./apps/example.cc)
target_link_libraries(dai-pipeline-example dai-pipeline-tools)

install(TARGETS dai-pipeline-tools DESTINATION lib)
install(TARGETS dai-pipeline-example DESTINATION bin)
install(FILES include/dai-pipeline-tools.h DESTINATION include)
