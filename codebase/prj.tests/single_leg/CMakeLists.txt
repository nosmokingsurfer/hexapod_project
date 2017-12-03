cmake_minimum_required(VERSION 3.1)

project(single_leg_test)
	
include_directories(
${CMAKE_SOURCE_DIR}/prj.corelibs/
)

set(single_leg_test_HDRS)
	
set(single_leg_test_SRC 
	src/single_leg_test.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)



add_executable(single_leg_test src/single_leg_test.cpp ${single_leg_test_HDRS} ${single_leg_test_SRC})
target_link_libraries (single_leg_test Eigen3::Eigen leg)