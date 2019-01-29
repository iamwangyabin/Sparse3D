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

/**
 * 只在图优化时候使用的信息帧类型
 * */
struct FrameInformation:FramedInformation{
    int flag;
    FrameInformation(int frame1, int frame2, const InformationMatrix& t)
    {
        frame1_=frame1;
        frame2_=frame2;
        information_=t;
        score_=0.0;
    }
    FrameInformation(int frame1, int frame2, const InformationMatrix& t, double score,int f)
    {
        frame1_=frame1;
        frame2_=frame2;
        information_=t;
        score_=score;
        flag=f;
    }

};

struct RGBDInformation2:RGBDInformation {
    std::vector< FrameInformation, Eigen::aligned_allocator<FrameInformation> > data_;

    void LoadFromFile(std::string filename) {
        data_.clear();
        int flag;
        int frame1, frame2;
        double score;
        InformationMatrix info;
        FILE * f = fopen(filename.c_str(), "r");
        if (f != NULL) {
            char buffer[1024];
            while (fgets(buffer, 1024, f) != NULL) {
                if (strlen(buffer) > 0 && buffer[0] != '#') {
                    sscanf(buffer, "%d %d %lf %d", &frame1, &frame2, &score, &flag);
                    fgets(buffer, 1024, f);
                    sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(0, 0), &info(0, 1), &info(0, 2), &info(0, 3), &info(0, 4), &info(0, 5));
                    fgets(buffer, 1024, f);
                    sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(1, 0), &info(1, 1), &info(1, 2), &info(1, 3), &info(1, 4), &info(1, 5));
                    fgets(buffer, 1024, f);
                    sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(2, 0), &info(2, 1), &info(2, 2), &info(2, 3), &info(2, 4), &info(2, 5));
                    fgets(buffer, 1024, f);
                    sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(3, 0), &info(3, 1), &info(3, 2), &info(3, 3), &info(3, 4), &info(3, 5));
                    fgets(buffer, 1024, f);
                    sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(4, 0), &info(4, 1), &info(4, 2), &info(4, 3), &info(4, 4), &info(4, 5));
                    fgets(buffer, 1024, f);
                    sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(5, 0), &info(5, 1), &info(5, 2), &info(5, 3), &info(5, 4), &info(5, 5));
                    data_.push_back(FrameInformation(frame1, frame2, info, score, flag));
                }
            }
            fclose(f);
        }
    }

    void SaveToFile(std::string filename) {
        FILE * f = fopen(filename.c_str(), "w");
        for (int i = 0; i < (int)data_.size(); i++) {
            InformationMatrix & info = data_[i].information_;
            fprintf(f, "%d\t%d\t%.8f\t%d\n", data_[i].frame1_, data_[i].frame2_, data_[i].score_, data_[i].flag);
            fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(0, 0), info(0, 1), info(0, 2), info(0, 3), info(0, 4), info(0, 5));
            fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(1, 0), info(1, 1), info(1, 2), info(1, 3), info(1, 4), info(1, 5));
            fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(2, 0), info(2, 1), info(2, 2), info(2, 3), info(2, 4), info(2, 5));
            fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(3, 0), info(3, 1), info(3, 2), info(3, 3), info(3, 4), info(3, 5));
            fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(4, 0), info(4, 1), info(4, 2), info(4, 3), info(4, 4), info(4, 5));
            fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(5, 0), info(5, 1), info(5, 2), info(5, 3), info(5, 4), info(5, 5));
        }
        fclose(f);
    }
};




/**=========================================================================================*/


struct SeqSaveAndLoad{
    template<typename T>
    bool Load(std::string filename, std::vector<T>& seq);

    template<typename T>
    bool Save(std::string filename, std::vector<T>& seq);

    template<typename T>
    bool Save(std::string filename, std::vector<T>& seq, RGBDTrajectory& loop);
};


#endif //SPARSE3D_COMMON_INCLUDE_H
