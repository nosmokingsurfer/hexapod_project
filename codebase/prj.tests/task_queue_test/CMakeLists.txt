cmake_minimum_required(VERSION 3.1)

project(task_queue_test)
	
include_directories(
${CMAKE_SOURCE_DIR}/prj.corelibs/
${CMAKE_SOURCE_DIR}/thirdparty/
)

set(task_queue_test_HDRS)
	
set(task_queue_test_SRC 
	src/task_queue_test.cpp
    ${CMAKE_SOURCE_DIR}/thirdparty/jsoncpp_headers/jsoncpp.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(task_queue_test src/task_queue_test.cpp ${task_queue_test_HDRS} ${task_queue_test_SRC} )
target_link_libraries (task_queue_test Eigen3::Eigen task_queue )
set_property(TARGET task_queue_test PROPERTY FOLDER "prj.tests")