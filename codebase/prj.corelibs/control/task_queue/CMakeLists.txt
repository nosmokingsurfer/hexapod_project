project(task_queue)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(task_queue_SRC src/task_queue.cpp)
set(task_queue_HDR task_queue.h)

add_library(task_queue ${task_queue_SRC} ${task_queue_HDR})

target_link_libraries (task_queue task Eigen3::Eigen)
set_property(TARGET task_queue PROPERTY FOLDER "prj.corelibs")