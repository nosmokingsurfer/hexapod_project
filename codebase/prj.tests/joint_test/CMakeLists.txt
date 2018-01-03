cmake_minimum_required(VERSION 3.1)

project(joint_test)
	
include_directories(
${CMAKE_SOURCE_DIR}/prj.corelibs/
)

set(joint_test_HDRS)
	
set(joint_test_SRC 
	src/joint_test.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)



add_executable(joint_test src/joint_test.cpp ${joint_test_HDRS} ${joint_test_SRC})
target_link_libraries (joint_test Eigen3::Eigen joint pose segment)
