//
// Created by wang on 19-1-16.
//

#ifndef SPARSE3D_COLORCORRESPONDENCE_H
#define SPARSE3D_COLORCORRESPONDENCE_H

#include "ColorGraphMatching.h"
#include "BuildCorpPointSet.h"
#include "common_include.h"

#include <pcl/filters/voxel_grid.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

void ConvertRGBCloud2XYZCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr rgb, pcl::PointCloud<pcl::PointXYZ>::Ptr xyz);

void ConvertXYZCloud2RGBCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr xyz, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb);

bool UVD2XYZ(int u, int v, unsigned short d, CameraParam& _camera, double& x, double& y, double& z);

pcl::PointXYZRGB  SearchNearestValidPoint(int u, int v, cv::Mat& depth, CameraParam& camera);

// correspondence
struct quad
{
    int image1_pixel_x, image1_pixel_y;
    int image2_pixel_x, image2_pixel_y;
};

struct CorrespondencePixelData
{
    int imageid1_, imageid2_;
    std::vector<quad> pixel_correspondence_;
};

struct CorrespondencePixel
{
    std::vector<CorrespondencePixelData> data_;
    void LoadFromFile(std::string filename);
};


int ColorCorrespondence(std::string pointcloud_dir, std::string pointcloud_ds_dir, std::string depth_dir, std::string correspendence_file, std::string camera_file,double score_max_depth, std::string traj_file, std::string info_file);


#endif //SPARSE3D_COLORCORRESPONDENCE_H
