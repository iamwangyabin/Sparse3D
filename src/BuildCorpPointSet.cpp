//
// Created by wang on 19-1-20.
//

#include "BuildCorpPointSet.h"

unsigned long BuildCorpPointSet::ComputeCorrepondencePointSet(const Eigen::Matrix4f &transform,
                                                              pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr scene1,
                                                              pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr scene2,
                                                              double score_max_depth) {
    _correspondences.clear();
    _correspondences.reserve(scene2->size());
    Eigen::Affine3f affine(transform);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*scene1, *scene, affine);

    pcl::KdTreeFLANN<pcl::PointXYZRGB> match_search;
    match_search.setInputCloud(scene2);

    int total_size = 0;

    const float max_range = _corr_dist_threshold * _corr_dist_threshold;
    for (int t = 0; t < scene->points.size(); ++t) {
        if (score_max_depth > 0 && scene1->points[t].z>score_max_depth)
            continue;
        total_size++;
        int N_nearest = 1;
        // 设置最临近点索引
        std::vector<int> neigh_indices(N_nearest);
        // 申明最临近平方距离值
        std::vector<float> neigh_sqr_dists(N_nearest);

        int found_neighs = match_search.nearestKSearch(scene->points.at(t), N_nearest, neigh_indices, neigh_sqr_dists);

        float dis = neigh_sqr_dists[0];
        if (dis > max_range)
            continue;

        pcl::Correspondence corr(t, neigh_indices[0], neigh_sqr_dists[0]);
        _correspondences.push_back(corr);
    }
    _total_size = total_size;
    return _correspondences.size();
}

Eigen::Matrix< double, 6, 6 > BuildCorpPointSet::ComputeInfoMatrix(pcl::Correspondences &cor,
                                                                   pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr scene1,
                                                                   pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr scene2) {
    Eigen::Matrix< double, 6, 6 > information_source;
    information_source.setZero();

    Eigen::Matrix< double, 6, 6 > information_target;
    information_target.setZero();

    for (int i=0;i<cor.size();++i){

    }
}