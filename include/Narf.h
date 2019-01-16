//
// Created by wang on 19-1-15.
//

#ifndef SPARSE3D_NARF_H
#define SPARSE3D_NARF_H

#include <vector>
#include <iostream>
#include <string>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image.h>

#include <boost/filesystem.hpp>

class NARF {

    pcl::RangeImage range_image;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr;
    pcl::PointCloud<int> keypoint_indices;

    float support_size = 0.08f;

public:
    pcl::PointCloud<pcl::PointXYZ> keypoints;
    pcl::PointCloud<pcl::Narf36> descriptors;

    NARF(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pointer);
    void narf_keypoints_extraction();

};

#endif //SPARSE3D_NARF_H
