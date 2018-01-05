cmake_minimum_required(VERSION 3.1)

project(mozaik_test)
	
include_directories(
${CMAKE_SOURCE_DIR}/prj.corelibs/
)

set(mozaik_test_HDRS)
	
set(mozaik_test_SRC 
	src/mozaik_test.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(mozaik_test src/mozaik_test.cpp ${mozaik_test_HDRS} ${mozaik_test_SRC})
target_link_libraries (mozaik_test Eigen3::Eigen mozaik body)
set_property(TARGET mozaik_test PROPERTY FOLDER "prj.tests")