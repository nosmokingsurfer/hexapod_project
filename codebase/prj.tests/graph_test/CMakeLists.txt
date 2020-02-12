cmake_minimum_required(VERSION 3.1)

project(graph_test)
	
include_directories(
${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(graph_test_SRC 
	src/graph_test.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)



add_executable(graph_test src/graph_test.cpp ${graph_test_SRC})
target_link_libraries (graph_test graph)
set_property(TARGET graph_test PROPERTY FOLDER "prj.tests")

add_test(graph_test graph_test)