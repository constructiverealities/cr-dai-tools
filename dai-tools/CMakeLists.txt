
set(CMAKE_CXX_STANDARD 17)

include_directories(${depthai_INCLUDE_DIRS} dai-tools/include)

if(NOT TARGET depthai::core)
  find_package(depthai REQUIRED)
  get_property(LIBNOP_INCLUDES TARGET libnop PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
endif()

include(CheckStructHasMember)

cmake_policy(SET CMP0026 OLD)
get_property(DEPTHAI_INCLUDES TARGET depthai::core PROPERTY INTERFACE_INCLUDE_DIRECTORIES)

set(CMAKE_REQUIRED_INCLUDES ${DEPTHAI_INCLUDES} ${LIBNOP_INCLUDES})
CHECK_STRUCT_HAS_MEMBER("class dai::node::ToF" inputConfig "depthai/pipeline/node/ToF.hpp" HAVE_CR_FORK LANGUAGE CXX)
message("Has CR Fork: ${HAVE_CR_FORK}")

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

add_library(cr-dai-tools STATIC src/cr-dai-tools.cc src/generate_pipeline.cc src/DepthToXYZ.cc)
target_link_libraries(cr-dai-tools depthai::core ${YAML_CPP_LIBRARIES})
 
target_include_directories(cr-dai-tools PUBLIC include ${depthai_INCLUDE_DIRS})

if(HAVE_CR_FORK)
  target_compile_options(cr-dai-tools PUBLIC "-DHAS_CR_FORK")
endif()

install(TARGETS cr-dai-tools
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include)
