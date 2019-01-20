//
// Created by wang on 19-1-20.
//

#ifndef SPARSE3D_BUILDCORPPOINTSET_H
#define SPARSE3D_BUILDCORPPOINTSET_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

class BuildCorpPointSet {
private:
    // 这个是论文里衡量两个点是不是重叠的量
    float _corr_dist_threshold;

public:
    // 从原始到目标点云的转换
    pcl::Correspondences _correspondences;
    pcl::Correspondences _final_correspondences;

    int _total_size;

    BuildCorpPointSet():_corr_dist_threshold(0.075),_total_size(0)
    {}

    // return correpondence number and correspondences point set
    unsigned long ComputeCorrepondencePointSet(const Eigen::Matrix4f& transform, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr scene1, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr scene2, double score_max_depth);

    Eigen::Matrix< double, 6, 6 > ComputeInfoMatrix(pcl::Correspondences& cor, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr scene1, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr scene2);

};
#endif //SPARSE3D_BUILDCORPPOINTSET_H
