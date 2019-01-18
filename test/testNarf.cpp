//
// Created by wang on 19-1-18.
//

#include "Narf.h"

int main(int argc, char** argv) {
    std::string pointcloud_xyzn_dir=argv[1];
    std::string keypoint_dir=argv[2];
    std::string descriptor_dir=argv[3];
    getNarf(pointcloud_xyzn_dir, keypoint_dir,descriptor_dir);
}