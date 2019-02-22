//
// Created by wang on 19-2-22.
//

#ifndef SPARSE3D_JOINTPOINTCLOUD_H
#define SPARSE3D_JOINTPOINTCLOUD_H

#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

struct FramedTransformation {
    int frame_;
    Eigen::Matrix4d transformation_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    FramedTransformation(int frame, const Eigen::Matrix4d& t)
            : frame_(frame), transformation_(t)
    {}
};


struct RGBDTrajectory {
    std::vector<FramedTransformation, Eigen::aligned_allocator<FramedTransformation>> data_;
    void LoadFromFile(std::string filename) ;
};


int showpcd(std::string filename);
void joinPointCloud(std::string pos_file,std::string pointcloud_dir);

#endif //SPARSE3D_JOINTPOINTCLOUD_H
