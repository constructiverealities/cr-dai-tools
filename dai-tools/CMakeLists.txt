
find_package(depthai REQUIRED)

include_directories(${depthai_INCLUDE_DIRS} dai-tools/include)
add_library(cr-dai-tools STATIC src/cr-dai-tools.cc src/generate_pipeline.cc)
target_link_libraries(cr-dai-tools depthai::core)
target_include_directories(cr-dai-tools PUBLIC include ${depthai_INCLUDE_DIRS})

install(TARGETS cr-dai-tools DESTINATION lib)
install(FILES dai-tools/include/cr-dai-tools.h DESTINATION dai-tools/include)
