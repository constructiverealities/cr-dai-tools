
set(CMAKE_CXX_STANDARD 17)

include_directories(${depthai_INCLUDE_DIRS} dai-tools/include)

if(NOT TARGET depthai::core)
  find_package(depthai REQUIRED)
endif()

if(NOT TARGET yaml-cpp)
  include(FetchContent)
  MESSAGE("Getting yaml")
  FetchContent_Declare(
          yaml-cpp
          GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
          GIT_TAG yaml-cpp-0.7.0
  )

  option(YAML_BUILD_SHARED_LIBS "Build yaml-cpp shared library" OFF)
  FetchContent_MakeAvailable(yaml-cpp)

  set(YAML_CPP_LIBRARIES yaml-cpp)
endif()

include_directories(${YAML_CPP_INCLUDE_DIR})

add_library(cr-dai-tools STATIC src/cr-dai-tools.cc src/generate_pipeline.cc)
target_link_libraries(cr-dai-tools depthai::core ${YAML_CPP_LIBRARIES})
 
target_include_directories(cr-dai-tools PUBLIC include ${depthai_INCLUDE_DIRS})

install(TARGETS cr-dai-tools
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include)
