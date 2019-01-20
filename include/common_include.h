//
// Created by wang on 19-1-16.
//

#ifndef SPARSE3D_COMMON_INCLUDE_H
#define SPARSE3D_COMMON_INCLUDE_H
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

struct CameraParam {
public:
    double fx_, fy_, cx_, cy_;
    double k1_, k2_, k3_, p1_, p2_;
    double depth_ratio_;
    double downsample_leaf;

    int img_width_;
    int img_height_;

    double integration_trunc_;

    CameraParam() :
            fx_(525.0f), fy_(525.0f), cx_(319.5f), cy_(239.5f),
            k1_(0.0), k2_(0.0), k3_(0.0), p1_(0.0), p2_(0.0),
            depth_ratio_(1000.0),
            downsample_leaf(0.05),
            img_width_(640), img_height_(480),
            integration_trunc_(4.0)
    {}

    void LoadFromFile(std::string filename);
};


struct KeypointDescriptor
{
    int keypoint_id;
    std::vector<float> dvec;
};

struct ImageDescriptor
{
    std::vector<KeypointDescriptor> data_;
    void LoadFromFile(std::string filename);
};

struct FramedTransformation {
    int frame1_;
    int frame2_;
    Eigen::Matrix4d transformation_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    FramedTransformation()
            :frame1_(-1), frame2_(-1), transformation_(Eigen::Matrix4d::Identity())
    {}
    FramedTransformation(int frame1,int frame2, const Eigen::Matrix4d& t)
            : frame1_(frame1), frame2_(frame2), transformation_(t)
    {}
};



/**
 * 轨迹
 * */
struct RGBDTrajectory {
    std::vector< FramedTransformation, Eigen::aligned_allocator<FramedTransformation>> data_;

    void LoadFromFile(std::string filename);
    void SaveToFile(std::string filename);

    void SaveToFileAppend(std::string filename);

    void SaveToSPFile(std::string filename);
};

typedef Eigen::Matrix< double, 6, 6, Eigen::RowMajor > InformationMatrix;

struct FramedInformation {
    int frame1_;
    int frame2_;
    InformationMatrix information_;

    double score_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    FramedInformation()
            :frame1_(-1), frame2_(-1), score_(-1.0)
    {
        information_ <<
                1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1;
    }
    FramedInformation(int frame1, int frame2, const InformationMatrix& t, double score)
            : frame1_(frame1), frame2_(frame2), information_(t), score_(score)
    {}
};


struct RGBDInformation {
    std::vector< FramedInformation, Eigen::aligned_allocator<FramedInformation> > data_;

    void LoadFromFile(std::string filename);
    void SaveToFile(std::string filename);
    void SaveToFileAppend(std::string filename);
    void SaveToSPFile(std::string filename);
};


#endif //SPARSE3D_COMMON_INCLUDE_H
