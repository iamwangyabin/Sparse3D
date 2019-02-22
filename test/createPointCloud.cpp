//
// Created by wang on 19-2-22.
//

#include "jointPointCloud.h"

int main(int argc, char** argv)
{
    std::string pos_file,pointcloud_dir;

    pos_file = argv[1];
    pointcloud_dir = argv[2];


//    showpcd(pointcloud_dir);
//    RGBDTrajectory traj;
//    traj.LoadFromFile(pointcloud_dir);
//    for (int i = 0; i < 10; ++i) {
////        Eigen::Matrix4d t = data_[i].transformation_;
//        FramedTransformation t = traj.data_[i];
//        cout<<t.frame_<<endl;
////        cout<<traj.data_.size();
//    }

    joinPointCloud(pos_file,pointcloud_dir);
    return (0);

}