cmake_minimum_required(VERSION 3.1)

project(articulated_test)
	
include_directories(
${CMAKE_SOURCE_DIR}/prj.corelibs/
)

set(articulated_test_HDRS)
	
set(articulated_test_SRC 
	src/articulated_test.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(articulated_test src/articulated_test.cpp ${articulated_test_HDRS} ${articulated_test_SRC})
target_link_libraries (articulated_test Eigen3::Eigen articulated body)
set_property(TARGET articulated_test PROPERTY FOLDER "prj.tests")