project(pid)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(pid_SRC src/pid.cpp)
set(pid_HDR pid.h)

add_library(pid ${pid_SRC} ${pid_HDR})

target_link_libraries (pid Eigen3::Eigen)