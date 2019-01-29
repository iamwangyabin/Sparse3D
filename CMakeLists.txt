cmake_minimum_required( VERSION 2.8 )
# Create Project
project( Sparse3D )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

# Set StartUp Project (Option)
# (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
#set_property( DIRECTORY PROPERTY "project" )
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )
#find_package(MLPACK REQUIRED)

# Find Packages
# Find PCL
#set(PCL_DIR /usr/include/pcl-1.7)
find_package( PCL 1.9 REQUIRED )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
# Find OpenCV
set(OpenCV_DIR /usr/local/share/OpenCV)
find_package( OpenCV 3.1 REQUIRED )
# Find Boost
find_package(Boost REQUIRED COMPONENTS
        system
        filesystem   # 我的工程中只使用了 boost 的 filesystem 功能,因此这里只有一个组件
        )
Find_Package(G2O REQUIRED)
Find_Package(Cholmod REQUIRED)
find_package(Armadillo REQUIRED)
find_package(BLAS)
find_package(LAPACK)
find_package(CSparse)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${ARMADILLO_LIBRARIES})

link_directories(${BLAS_LIBRARIES})
link_directories(${LAPACK_LIBRARIES})

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear
        g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface
        g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense
        g2o_stuff g2o_types_sclam2d g.2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

#set(PCL_INCLUDE_DIRS /usr/include/pcl-1.7)
set( THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ${BLAS_LIBRARIES}
        ${ARMADILLO_LIBRARIES}
        ${CSPARSE_LIBRARY}
        )
add_definitions(${PCL_DEFINITIONS})

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        "/usr/include/eigen3/"
        ${Boost_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${ARMADILLO_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
)

add_subdirectory( src )
add_subdirectory( test )