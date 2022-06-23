cmake_minimum_required(VERSION 3.0.2)
project(xenomai_test)

add_compile_options(-O)

# execute_process(COMMAND xeno-config --posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

# execute_process(COMMAND xeno-config --posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

find_package(dwbc REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


include_directories(
  include
)
add_library(realtime_lib src/realtime.cpp)

add_executable(xenomai_test src/main.cpp)

target_link_libraries(xenomai_test
# ${XENO_CFLAGS}
# ${XENO_LDFLAGS}
dwbc
realtime_lib
Eigen3::Eigen
pthread
)
