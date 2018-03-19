cmake_minimum_required(VERSION 3.1)

project(linear_player_test)
	
include_directories(
${CMAKE_SOURCE_DIR}/prj.corelibs/
)

set(linear_player_test_HDRS)
	
set(linear_player_test_SRC 
	src/linear_player_test.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(linear_player_test src/linear_player_test.cpp ${linear_player_test_HDRS} ${linear_player_test_SRC})
target_link_libraries (linear_player_test Eigen3::Eigen linear_player )
set_property(TARGET linear_player_test PROPERTY FOLDER "prj.tests")