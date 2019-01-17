//
// Created by wang on 19-1-16.
//

#include "GraphMatching.h"


GraphMatching::GraphMatching(
        pcl::PointCloud<pcl::PointXYZRGB>& keypoints1,
        pcl::PointCloud<pcl::PointXYZRGB>& keypoints2,
        const pcl::Correspondences& correspondence)
        :_m(keypoints1.size()*keypoints2.size(), keypoints1.size()*keypoints2.size()),
         _keypoints1(keypoints1), _keypoints2(keypoints2),
         _nrow(static_cast<int>(keypoints1.size() * keypoints2.size())),
         _ncol(static_cast<int>(keypoints1.size() * keypoints2.size()))
{
    double threshold = 0.01;

    for (int i = 0; i < correspondence.size(); ++i)
    {
        for (int j = i + 1; j < correspondence.size(); ++j)
        {
            int model1_kp1 = correspondence[i].index_query;
            int model1_kp2 = correspondence[j].index_query;
            int model2_kp1 = correspondence[i].index_match;
            int model2_kp2 = correspondence[j].index_match;

            if (model1_kp1 == model1_kp2 || model2_kp1 == model2_kp2)
                continue;

            int index_ij1 = static_cast<int>(model1_kp1 * keypoints2.size() + model2_kp1);
            int index_ij2 = static_cast<int>(model1_kp2 * keypoints2.size() + model2_kp2);
            assert(index_ij1 != index_ij2);

            float dis_model1_x = keypoints1.points[model1_kp1].x - keypoints1.points[model1_kp2].x;
            float dis_model1_y = keypoints1.points[model1_kp1].y - keypoints1.points[model1_kp2].y;
            float dis_model1_z = keypoints1.points[model1_kp1].z - keypoints1.points[model1_kp2].z;
            if (fabs(dis_model1_x) < 0.01 && fabs(dis_model1_y) < 0.01 && fabs(dis_model1_z) < 0.01)
                continue;

            float dis_model2_x = keypoints2.points[model2_kp1].x - keypoints2.points[model2_kp2].x;
            float dis_model2_y = keypoints2.points[model2_kp1].y - keypoints2.points[model2_kp2].y;
            float dis_model2_z = keypoints2.points[model2_kp1].z - keypoints2.points[model2_kp2].z;
            if (fabs(dis_model2_x) < 0.01 && fabs(dis_model2_y) < 0.01 && fabs(dis_model2_z) < 0.01)
                continue;

            float dis_model1 = sqrt(dis_model1_x*dis_model1_x + dis_model1_y*dis_model1_y + dis_model1_z*dis_model1_z);
            float dis_model2 = sqrt(dis_model2_x*dis_model2_x + dis_model2_y*dis_model2_y + dis_model2_z*dis_model2_z);

            if (fabs(dis_model1 - dis_model2) >= 3 * threshold)
                continue;

            int row = index_ij1;
            int col = index_ij2;
            float val = 4.5 - (dis_model1 - dis_model2)*(dis_model1 - dis_model2) / (2 * threshold*threshold);

            _m(row, col) = val;
            _m(col, row) = val;

        }
    }
}