
find_package(depthai REQUIRED)

find_path(HAS_TOF_SUPPORT "ToF.hpp" PATHS ${depthai_DIR}/../../../include/depthai/pipeline/node)
if(HAS_TOF_SUPPORT)
    add_compile_options("-DDEPTHAI_HAS_TOF_NODE=1")
    message("HAS TOF SUPPORT")
else()
    message("HAS NO TOF SUPPORT")
endif()

include_directories(${depthai_INCLUDE_DIRS} dai-tools/include)
add_library(cr-dai-tools STATIC src/cr-dai-tools.cc src/generate_pipeline.cc)
target_link_libraries(cr-dai-tools depthai::core)
target_include_directories(cr-dai-tools PUBLIC include)

install(TARGETS cr-dai-tools DESTINATION lib)
install(FILES dai-tools/include/cr-dai-tools.h DESTINATION dai-tools/include)
