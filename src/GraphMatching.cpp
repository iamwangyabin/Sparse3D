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

            float dis_model1 = static_cast<float>(sqrt(dis_model1_x * dis_model1_x + dis_model1_y * dis_model1_y + dis_model1_z * dis_model1_z));
            float dis_model2 = static_cast<float>(sqrt(dis_model2_x * dis_model2_x + dis_model2_y * dis_model2_y + dis_model2_z * dis_model2_z));

            if (fabs(dis_model1 - dis_model2) >= 3 * threshold)
                continue;

            int row = index_ij1;
            int col = index_ij2;
            float val = static_cast<float>(4.5 - (dis_model1 - dis_model2) * (dis_model1 - dis_model2) / (2 * threshold * threshold));

            _m(row, col) = val;
            _m(col, row) = val;

        }
    }
}
bool GraphMatching::judge_pointn_plane(std::vector<pcl::PointXYZ>& arr)
{
    Eigen::Vector3f v1, v2, v3;
    v1 <<
       arr[1].x - arr[0].x, arr[1].y - arr[0].y, arr[1].z - arr[0].z;
    v2 <<
       arr[2].x - arr[0].x, arr[2].y - arr[0].y, arr[2].z - arr[0].z;

    Eigen::Vector3f n;
    n = v1.cross(v2);
    n.normalize();

    for (int i = 3; i < arr.size(); ++i)
    {
        v3 <<
           arr[i].x - arr[0].x, arr[i].y - arr[0].y, arr[i].z - arr[0].z;
        float dis = v3.dot(n);
        dis = fabs(dis);

        float the_same_plane_threshold = 0.11f;

        if (dis > the_same_plane_threshold)
        {
            return false;
        }
    }
    return true;
}