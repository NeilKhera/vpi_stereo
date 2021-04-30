cmake_minimum_required(VERSION 3.10)
project(vpi_utils CXX)

find_package(OpenCV REQUIRED)
find_package(vpi REQUIRED)

add_compile_options(-Wall -Wextra -pedantic)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(vpi_utils_library src/Disparity.cpp include/Disparity.h)
target_include_directories(vpi_utils_library PRIVATE src PUBLIC include)

# It uses vpi and opencv. CMake will automatically
# set up the correct header and library directories,
# and make hello_work link to these libraries.
target_link_libraries(vpi_utils_library PRIVATE vpi ${OpenCV_LIBS})
