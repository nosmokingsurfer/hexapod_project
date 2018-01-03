cmake_minimum_required(VERSION 3.1)

project(trajectory_test)
	
include_directories(
${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(trajectory_test_SRC 
	src/trajectory_test.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)



add_executable(trajectory_test src/trajectory_test.cpp ${trajectory_test_SRC})
target_link_libraries (trajectory_test Eigen3::Eigen trajectory pose)