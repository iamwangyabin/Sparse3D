//
// Created by wang on 19-1-16.
//

#ifndef SPARSE3D_COLORCORRESPONDENCE_H
#define SPARSE3D_COLORCORRESPONDENCE_H

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




#endif //SPARSE3D_COLORCORRESPONDENCE_H
