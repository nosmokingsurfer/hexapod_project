project(linear_player)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(linear_player_SRC src/linear_player.cpp)
set(linear_player_HDR linear_player.h)

add_library(linear_player ${linear_player_SRC} ${linear_player_HDR})

target_link_libraries (linear_player Eigen3::Eigen)
set_property(TARGET linear_player PROPERTY FOLDER "prj.corelibs")