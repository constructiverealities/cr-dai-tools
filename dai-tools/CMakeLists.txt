
set(CMAKE_CXX_STANDARD 17)

include_directories(${depthai_INCLUDE_DIRS} dai-tools/include)

if(NOT TARGET depthai::core)
  find_package(depthai REQUIRED)
endif()

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

add_library(cr-dai-tools STATIC src/cr-dai-tools.cc src/generate_pipeline.cc)
target_link_libraries(cr-dai-tools depthai::core ${YAML_CPP_LIBRARIES})
 
target_include_directories(cr-dai-tools PUBLIC include ${depthai_INCLUDE_DIRS})

install(TARGETS cr-dai-tools
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include)
