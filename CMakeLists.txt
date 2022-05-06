cmake_minimum_required(VERSION 3.10)
project(cr-dai-tools)

set(CMAKE_CXX_STANDARD 17)
include(CheckCXXSourceCompiles)

#add_compile_options(-fsanitize=address)
#add_link_options(-fsanitize=address)

#set(SHARED_FLAGS "-fPIC -Wall -Wno-unused-variable -Wno-switch -Wno-parentheses -Wno-missing-braces -Werror=return-type -Werror=vla -fno-math-errno -Werror=pointer-arith")
#
#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${SHARED_FLAGS} -std=gnu99 -Werror=incompatible-pointer-types -Werror=implicit-function-declaration ")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SHARED_FLAGS} -std=c++14")

find_package(depthai REQUIRED)
add_subdirectory(dai-tools)
add_subdirectory(ros)

add_executable(cr-dai-example ./apps/example.cc)
target_link_libraries(cr-dai-example cr-dai-tools)

install(TARGETS cr-dai-example DESTINATION bin)
