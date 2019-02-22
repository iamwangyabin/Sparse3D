#add_definitions(
#        ${PCL_DEFINITIONS})
#
#add_executable( sift sift_test.cpp )
#target_link_libraries( sift Sparse3D )
#
#add_executable( Png2Cloud testPng2Cloud.cpp )
#target_link_libraries( Png2Cloud Sparse3D )

#add_executable( readPara readPara.cpp )
#target_link_libraries( readPara Sparse3D )

#add_executable( ShiTomasi testShiTomasi.cpp )
#target_link_libraries( ShiTomasi Sparse3D )
#
#add_executable( Narf testNarf.cpp )
#target_link_libraries( Narf Sparse3D )
#
#add_executable( Pcl1 testPcl.cpp )
#target_link_libraries( Pcl1 Sparse3D )
#
#add_executable( Pcl narf_keypoint_extration.cpp )
#target_link_libraries( Pcl Sparse3D )

#add_executable( ColorCor ColorCor.cpp)
#target_link_libraries( ColorCor Sparse3D )

#add_executable( GeoCorr testGeometricCorr.cpp)
#target_link_libraries( GeoCorr Sparse3D )

#add_executable( Opt1 testOpt1.cpp)
#target_link_libraries( Opt1 Sparse3D )

add_executable( createPointCloud createPointCloud.cpp)
target_link_libraries( createPointCloud Sparse3D )

