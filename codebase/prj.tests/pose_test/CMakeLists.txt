cmake_minimum_required(VERSION 3.1)

project(pose_test)
	
include_directories(
${CMAKE_SOURCE_DIR}/prj.corelibs/
)

set(pose_test_HDRS)
	
set(pose_test_SRC 
	src/pose_test.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)



add_executable(pose_test src/pose_test.cpp ${pose_test_HDRS} ${pose_test_SRC})
target_link_libraries (pose_test Eigen3::Eigen pose)
set_property(TARGET pose_test PROPERTY FOLDER "prj.tests")