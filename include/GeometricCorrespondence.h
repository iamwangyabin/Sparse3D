//
// Created by wang on 19-1-20.
//

#ifndef SPARSE3D_GEOMETRICCORRESPONDENCE_H
#define SPARSE3D_GEOMETRICCORRESPONDENCE_H

#include <vector>
#include <iostream>
#include <string>
#include <set>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/correspondence.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <boost/filesystem.hpp>

//#include "common_include.h"

#include "GeoSubsidiary.h"

int GeoCorrespondence(
        std::string pointcloud_dir,
        std::string pointcloud_ds_dir,
        std::string keypoint_dir,
        std::string descriptor_dir,
        std::string camera_file,
        std::string traj_file,
        std::string info_file,
        double score_max_depth);

#endif //SPARSE3D_GEOMETRICCORRESPONDENCE_H
