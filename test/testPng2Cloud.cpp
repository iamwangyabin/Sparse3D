//
// Created by wang on 19-1-15.
//

#include "Png2Cloud.h"

int main(int argc, char** argv){
    std::string depth_dir = argv[1];
    std::string color_dir = argv[2];
    std::string camera_para = argv[3];
    std::string pointcloud_dir = argv[4];
    std::string pointcloud_ds_dir = argv[5];
    std::string xyzn_dir = argv[6];
    std::string xyzn_ds_dir = argv[7];

    ChangePng2Cloud(depth_dir,color_dir ,camera_para ,pointcloud_dir ,pointcloud_ds_dir ,xyzn_dir ,xyzn_ds_dir );
}