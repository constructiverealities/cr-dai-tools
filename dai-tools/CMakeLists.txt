
set(CMAKE_CXX_STANDARD 17)

find_package(depthai REQUIRED)

include_directories(${depthai_INCLUDE_DIRS} dai-tools/include)
add_library(cr-dai-tools STATIC src/cr-dai-tools.cc src/generate_pipeline.cc)
target_link_libraries(cr-dai-tools depthai::core yaml-cpp)
target_include_directories(cr-dai-tools PUBLIC include ${depthai_INCLUDE_DIRS})

install(TARGETS cr-dai-tools
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include)
