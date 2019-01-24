//
// Created by wang on 19-1-16.
//

#ifndef SPARSE3D_GRAPHMATCHING_H
#define SPARSE3D_GRAPHMATCHING_H

#include <iostream>
#include <vector>
#include <string>
#include <cassert>
#include <fstream>
#include <algorithm>

// 这个库需要编译安装一下
#include <armadillo>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/common/transforms.h>

#include <Eigen/Eigen>


class GraphMatching
{

public:
    int _nrow;
    int _ncol;
    arma::sp_fmat _m;   // 稀疏矩阵


    pcl::PointCloud<pcl::PointXYZRGB>& _keypoints1;
    pcl::PointCloud<pcl::PointXYZRGB>& _keypoints2;
    // final transformation between two frames
    Eigen::Matrix4f		_transformation;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    GraphMatching(pcl::PointCloud<pcl::PointXYZRGB>& keypoints1, pcl::PointCloud<pcl::PointXYZRGB>& keypoints2, const pcl::Correspondences& correspondence);
//    GraphMatching(pcl::PointCloud<pcl::PointXYZ>& keypoints1, pcl::PointCloud<pcl::PointXYZ>& keypoints2, const pcl::Correspondences& correspondence);
//    GraphMatching(pcl::PointCloud<T>& keypoints1, pcl::PointCloud<T>& keypoints2, const pcl::Correspondences& correspondence);

//    GraphMatching(pcl::PointCloud<T> &_keypoints1);
    // refine correspondences
    /**
     * 就是找到一个变换矩阵 4x4 的，包含旋转和平移
     * */
    pcl::CorrespondencesPtr ComputeCorrespondenceByEigenVec(int best_num = 4);
    // give n pairs correspondences, compute rigid transformation
    /**
     * 刚体变换（Rigid Transformation）
     * */
    Eigen::Matrix4f ComputeRigid(pcl::CorrespondencesConstPtr correspondence, const pcl::PointCloud<pcl::PointXYZRGB>& keypoints1, const pcl::PointCloud<pcl::PointXYZRGB>& keypoints2);
//    Eigen::Matrix4f	ComputeRigid(pcl::CorrespondencesConstPtr correspondence, const pcl::PointCloud<pcl::PointXYZ>& keypoints1, const pcl::PointCloud<pcl::PointXYZ>& keypoints2);
//    Eigen::Matrix4f ComputeRigid(pcl::CorrespondencesConstPtr correspondence, const pcl::PointCloud<T>& keypoints1, const pcl::PointCloud<T>& keypoints2);

private:
    // judge n points in the same plane or not
    bool judge_pointn_plane(std::vector<pcl::PointXYZ>& arr);
};


#endif //SPARSE3D_GRAPHMATCHING_H
