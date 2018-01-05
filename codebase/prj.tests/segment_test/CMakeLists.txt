cmake_minimum_required(VERSION 3.1)

project(segment_test)
	
include_directories(
${CMAKE_SOURCE_DIR}/prj.corelibs/
)

set(segment_test_HDRS)
	
set(segment_test_SRC 
	src/segment_test.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(segment_test src/segment_test.cpp ${segment_test_HDRS} ${segment_test_SRC})
target_link_libraries (segment_test Eigen3::Eigen leg segment joint )
set_property(TARGET segment_test PROPERTY FOLDER "prj.tests")