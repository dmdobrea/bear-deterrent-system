cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(bearDetection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_PREFIX_PATH "/opt/libtorch-cuda")

find_package(Torch REQUIRED)
find_package(OpenCV REQUIRED)

# Find libcurl
find_package(CURL REQUIRED)

# Enable debug symbols by default
#if(NOT CMAKE_BUILD_TYPE)
#    set(CMAKE_BUILD_TYPE Release)   #Debug
#endif()

add_executable(bearDetection bear_detection.cpp)

# Add extra debug flags if needed
#target_compile_options(bearDetection PRIVATE
#    $<$<CONFIG:Debug>:-g -O0>
#)

# Link Torch, OpenCV, and libcurl
target_link_libraries(bearDetection 
	"${TORCH_LIBRARIES}" 
	${OpenCV_LIBS}
	CURL::libcurl
	)

set_property(TARGET bearDetection PROPERTY CXX_STANDARD 17)

# print build type during configuration
#message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
