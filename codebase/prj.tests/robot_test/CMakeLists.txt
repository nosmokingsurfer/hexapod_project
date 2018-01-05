cmake_minimum_required(VERSION 3.1)

project(robot_test)
	
include_directories(
${CMAKE_SOURCE_DIR}/prj.corelibs/
)

set(robot_test_HDRS)
	
set(robot_test_SRC 
	src/robot_test.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)



add_executable(robot_test ${robot_test_HDRS} ${robot_test_SRC})
target_link_libraries (robot_test Eigen3::Eigen leg robot)
set_property(TARGET robot_test PROPERTY FOLDER "prj.tests")