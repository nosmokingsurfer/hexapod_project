cmake_minimum_required(VERSION 3.1)

project(leg_test)
	
include_directories(
${CMAKE_SOURCE_DIR}/prj.corelibs/
)

set(leg_test_HDRS)
	
set(leg_test_SRC 
	src/leg_test.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)



add_executable(leg_test src/leg_test.cpp ${leg_test_HDRS} ${leg_test_SRC})
target_link_libraries (leg_test Eigen3::Eigen leg pose)
set_property(TARGET leg_test PROPERTY FOLDER "prj.tests")