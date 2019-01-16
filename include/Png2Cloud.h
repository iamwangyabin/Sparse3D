//
// Created by wang on 19-1-15.
//

/**
 * 这个是生成云图的程序
 *
 * */

#ifndef SPARSE3D_PNG2CLOUD_H
#define SPARSE3D_PNG2CLOUD_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <boost/filesystem.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>

#include "common_include.h"

int ChangePng2Cloud(    std::string depth_dir ,
                        std::string color_dir ,
                        std::string camera_para ,
                        std::string pointcloud_dir ,
                        std::string pointcloud_ds_dir ,
                        std::string xyzn_dir ,
                        std::string xyzn_ds_dir);


class Png2Cloud
{
public:
    CameraParam _camera;

    bool Load(int png_index, std::string& depth_dir, std::string& color_dir, cv::Mat& depth_img, cv::Mat& color_img);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Png2RGBcloud(cv::Mat& depth_img, cv::Mat& color_img);

    void ConvertRGBCloud2XYZCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr rgb, pcl::PointCloud<pcl::PointXYZ>::Ptr xyz);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ComputeModelNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, float radius=0.05);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr DownSamle(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_in);


    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr DownSamle(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr cloud_in);

    int round(double x) {
        return static_cast<int>(floor(x + 0.5));
    }

    bool UVD2XYZ(int u, int v, unsigned short d, double & x, double & y, double & z);

    bool XYZ2UVD(double x, double y, double z, int & u, int & v, unsigned short & d);
};

#endif //SPARSE3D_PNG2CLOUD_H