project(graph)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(graph_SRC src/graph.cpp)
set(graph_HDR graph.h)

add_library(graph ${graph_SRC} ${graph_HDR})

target_link_libraries (graph pose)
set_property(TARGET graph PROPERTY FOLDER "prj.corelibs")