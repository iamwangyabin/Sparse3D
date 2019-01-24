//
// Created by wang on 19-1-24.
//

#include "ColorCorrespondence.h"
#include "ColorGraphMatching.h"

using namespace std;

int main(int argc, char** argv) {
    double score_max_depth;
    std::string pointcloud_dir, pointcloud_ds_dir, depth_dir, correspendence_file, traj_file, info_file, camera_file;

    pointcloud_dir = argv[1];
    pointcloud_ds_dir = argv[2];
    depth_dir = argv[3];
    correspendence_file = argv[4];
    camera_file = argv[5];
    score_max_depth = atof(argv[6]);

    traj_file = argv[7];
    info_file = argv[8];

    ColorCorrespondence(pointcloud_dir, pointcloud_ds_dir, depth_dir, correspendence_file, camera_file, score_max_depth, traj_file, info_file);

}
