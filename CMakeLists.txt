cmake_minimum_required(VERSION 2.8)
project(bal)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(G2O REQUIRED)
include_directories( ${G2O_INCLUDE_DIRS})
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear 
    g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface 
    g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense 
    g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

include_directories("/usr/include/eigen3")
message("find g2o libraries in: " ${G2O_INCLUDE_DIR})
message("find g2o lib in: " ${G2O_LIBS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
include_directories( ${PROJECT_SOURCE_DIR}/include)
add_library(problem SHARED src/problem.cpp)

add_executable(bal bal.cpp)
target_link_libraries(bal ${G2O_LIBS} problem)


