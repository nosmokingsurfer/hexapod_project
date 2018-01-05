cmake_minimum_required(VERSION 3.1)

project(step_cycle_test)
	
include_directories(
${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(step_cycle_test_HDRS)

set(step_cycle_test_SRC 
	src/step_cycle_test.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)



add_executable(step_cycle_test src/step_cycle_test.cpp ${step_cycle_test_HDRS} ${step_cycle_test_SRC})
target_link_libraries (step_cycle_test Eigen3::Eigen step)
set_property(TARGET step_cycle_test PROPERTY FOLDER "prj.tests")