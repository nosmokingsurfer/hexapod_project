project(task)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
    ${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(task_SRC src/task.cpp)
set(task_HDR task.h)

add_library(task ${task_SRC} ${task_HDR})

target_link_libraries (task pose Eigen3::Eigen)
set_property(TARGET task PROPERTY FOLDER "prj.corelibs")