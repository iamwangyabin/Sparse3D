#add_definitions(
#        ${PCL_DEFINITIONS})

add_executable( sift sift_test.cpp )
target_link_libraries( sift Sparse3D )

#add_executable( testPng2Cloud testPng2Cloud.cpp )
#target_link_libraries( testPng2Cloud Sparse3D )
#
#add_executable( readPara readPara.cpp )
#target_link_libraries( readPara Sparse3D )