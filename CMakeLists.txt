


cmake_minimum_required(VERSION 3.10)

project(Mesh_polyhedral_ply_demo)
find_package(CGAL REQUIRED)
find_package(PCL 1.8 REQUIRED)

include(${CGAL_USE_FILE})
include_directories(${PCL_INCLUDE_DIRS})


link_directories(${PCL_LIBRARY_DIRS})
link_directories(${CGAL_LIBRARY_DIRS})

add_executable (Mesh_polyhedral_ply_demo Mesh_polyhedral_ply_demo.cpp)
target_link_libraries (Mesh_polyhedral_ply_demo ${PCL_LIBRARIES}  ${CGAL_LIBS})

