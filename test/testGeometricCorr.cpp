//
// Created by wang on 19-2-20.
//

//
// Created by wang on 19-1-24.
//

#include "GeometricCorrespondence.h"
//#include "GeoGraphMatching.h"
//
using namespace std;

int main(int argc, char** argv) {
    double score_max_depth;
    std::string pointcloud_dir;
    std::string pointcloud_ds_dir;
    std::string keypoint_dir;
    std::string descriptor_dir;
    std::string camera_file;
    std::string traj_file;
    std::string info_file;

    pointcloud_dir = argv[1];
    pointcloud_ds_dir = argv[2];
    keypoint_dir = argv[3];
    descriptor_dir = argv[4];
    camera_file = argv[5];
    score_max_depth = atof(argv[6]);

    traj_file = argv[7];
    info_file = argv[8];
    GeoCorrespondence(pointcloud_dir,pointcloud_ds_dir,keypoint_dir,descriptor_dir,camera_file,traj_file,info_file,score_max_depth);
}