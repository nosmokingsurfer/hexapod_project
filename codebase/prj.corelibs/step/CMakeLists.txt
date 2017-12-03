project(step)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(step_SRC src/step.cpp)
set(step_HDR step.h)

add_library(step ${step_SRC} ${step_HDR})

target_link_libraries (step Eigen3::Eigen)