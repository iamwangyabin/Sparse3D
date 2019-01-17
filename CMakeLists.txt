cmake_minimum_required( VERSION 2.8 )
# Create Project
project( Sparse3D )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

# Set StartUp Project (Option)
# (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
#set_property( DIRECTORY PROPERTY "project" )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )


# Find Packages
# Find PCL
find_package( PCL 1.7 REQUIRED )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
# Find OpenCV
set(OpenCV_DIR /usr/local/share/OpenCV)
find_package( OpenCV 3.1 REQUIRED )
# Find Boost
find_package(Boost REQUIRED COMPONENTS
        system
        filesystem   # 我的工程中只使用了 boost 的 filesystem 功能,因此这里只有一个组件
        )
find_package(Armadillo REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})


#set(PCL_INCLUDE_DIRS /usr/include/pcl-1.7)
set( THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        )


include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        "/usr/include/eigen3/"
        ${Boost_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${ARMADILLO_INCLUDE_DIRS}
)

add_subdirectory( src )
add_subdirectory( test )