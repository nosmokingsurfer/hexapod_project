project(robot)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(robot_SRC src/robot.cpp)
set(robot_HDR robot.h)

add_library(robot ${robot_SRC} ${robot_HDR})

target_link_libraries(robot trajectory leg pid body Eigen3::Eigen)
SET_PROPERTY(TARGET robot PROPERTY FOLDER "prj.corelibs")