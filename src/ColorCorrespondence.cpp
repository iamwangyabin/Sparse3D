//
// Created by wang on 19-1-16.
//
#include "common_include.h"
#include "ColorCorrespondence.h"

int ColorCorrespondence(char** argv){
    double score_max_depth;
    std::string pointcloud_dir, pointcloud_ds_dir, depth_dir, correspendence_file, traj_file, info_file, camera_file;

    pointcloud_dir = argv[1];
    pointcloud_ds_dir = argv[2];
    depth_dir = argv[3];
    correspendence_file = argv[4];
    camera_file = argv[5];
    score_max_depth = atof(argv[6]);

    traj_file = argv[7];
    info_file = argv[8];

    int num_of_pc = static_cast<int>(std::count_if(
        boost::filesystem::directory_iterator(boost::filesystem::path(depth_dir)),
        boost::filesystem::directory_iterator(),
        [](const boost::filesystem::directory_entry& e) {
        return e.path().extension() == ".png";  }));

    CameraParam camera;
    camera.LoadFromFile(camera_file);

    CorrespondencePixel cor_pixel;
    cor_pixel.LoadFromFile(correspendence_file);

    // Load all downsample pointcloud
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> downsample_pc;
    for (int i = 0; i < num_of_pc;++i)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_ds(new pcl::PointCloud<pcl::PointXYZRGB>);

        std::stringstream dsss1;
        dsss1 << pointcloud_ds_dir << "pointcloud_ds" << i << ".pcd";
        pcl::io::loadPCDFile(dsss1.str(), *scene_ds);
        downsample_pc.push_back(scene_ds);
    }

    // Load all depth images
    std::vector<cv::Mat> depth_img;
    for (int i = 0; i < num_of_pc;++i)
    {
        std::stringstream ss1;
        ss1 << depth_dir << "depth" << i << ".png";
        cv::Mat depth_image = cv::imread(ss1.str(), CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
        depth_image.convertTo(depth_image, CV_16U);
        depth_img.push_back(depth_image);
    }

    RGBDInformation info;	info.data_.resize(num_of_pc * num_of_pc);
    RGBDTrajectory  traj;	traj.data_.resize(num_of_pc * num_of_pc);

    if (boost::filesystem::exists(traj_file))
        boost::filesystem::remove(traj_file);
    if (boost::filesystem::exists(info_file))
        boost::filesystem::remove(info_file);

#pragma omp parallel for num_threads( 8 ) schedule( dynamic )
    for (int i = 0; i < cor_pixel.data_.size(); ++i) {
        int img1 = cor_pixel.data_[i].imageid1_;
        int img2 = cor_pixel.data_[i].imageid2_;
        cv::Mat& depth_image1 = depth_img[img1];
        cv::Mat& depth_image2 = depth_img[img2];

        // prepare
        pcl::CorrespondencesPtr corps(new pcl::Correspondences);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_keypoints1(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_keypoints2(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int t = 0; t < cor_pixel.data_[i].pixel_correspondence_.size(); ++t)
        {
            int image1_u = cor_pixel.data_[i].pixel_correspondence_[t].image1_pixel_x;
            int image1_v = cor_pixel.data_[i].pixel_correspondence_[t].image1_pixel_y;
            pcl::PointXYZRGB p1 = SearchNearestValidPoint(image1_u, image1_v, depth_image1, camera);

            int image2_u = cor_pixel.data_[i].pixel_correspondence_[t].image2_pixel_x;
            int image2_v = cor_pixel.data_[i].pixel_correspondence_[t].image2_pixel_y;
            pcl::PointXYZRGB p2 = SearchNearestValidPoint(image2_u, image2_v, depth_image2, camera);

            if (p1.z < 0.0 || p2.z < 0.0)
                continue;

            pointcloud_keypoints1->push_back(p1);
            pointcloud_keypoints2->push_back(p2);

            pcl::Correspondence c;
            c.index_query = static_cast<int>(pointcloud_keypoints1->size() - 1);
            c.index_match = static_cast<int>(pointcloud_keypoints2->size() - 1);
            corps->push_back(c);
        }
        if (corps->size() < 4)
            continue;
        /**
         * 以上实际上生成了一组对应点，肯定有很多错配大的点，但是不重要，后面可以消除这些点影响。
         * 4就是说要想求出单应矩阵起码需要4个点
         * 以下要求出变换
         * 就是论文里4.3的东西
         * */
        //Graph matching
        GraphMatching gm(*pointcloud_keypoints1, *pointcloud_keypoints2, *corps);
        pcl::CorrespondencesPtr	graph_corps = gm.ComputeCorrespondenceByEigenVec();

    }
}


/**
 * 这东西需要返回的data数据类型就是一组数据结构
 * 其中每一项由对应的两个图片地址1,2+四个坐标组成
 * 应该从correspondens.txt文件下读取很多行
 */
void CorrespondencePixel::LoadFromFile(std::string filename) {
    std::ifstream input(filename);
    if (!input.good()){
        std::cerr << "Can't open file " << filename << "!" << std::endl;
        exit(-1);
    }

    while (input.good()){
        std::string line;
        std::getline(input, line);
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string keyword;
        ss >> keyword;
        if (keyword == "#")
            continue;
        if (keyword == "image_id")
        {
            int id1, id2;
            ss >> id1 >> id2;

            CorrespondencePixelData data;
            data.imageid1_ = id1;
            data.imageid2_ = id2;

            data_.push_back(data);

            continue;
        }

        int a, b, c, d;
        a = atoi(keyword.c_str());
        ss >> b >> c >> d;

        quad q{};
        q.image1_pixel_x = a;	q.image1_pixel_y = b;
        q.image2_pixel_x = c;	q.image2_pixel_y = d;
        data_[data_.size() - 1].pixel_correspondence_.push_back(q);
    }

}


/**
 * RGBDTrajectory::RGBDTrajectory::RGBDTrajectory::RGBDTrajectory::RGBDTrajectory::
 * */
void RGBDTrajectory::LoadFromFile(std::string filename) {
    data_.clear();
    int frame1, frame2;
    Eigen::Matrix4d trans;
    FILE * f = fopen(filename.c_str(), "r");
    if (f != NULL) {
        char buffer[1024];
        while (fgets(buffer, 1024, f) != NULL) {
            if (strlen(buffer) > 0 && buffer[0] != '#') {
                sscanf(buffer, "%d %d", &frame1, &frame2);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf %lf %lf %lf", &trans(0, 0), &trans(0, 1), &trans(0, 2), &trans(0, 3));
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf %lf %lf %lf", &trans(1, 0), &trans(1, 1), &trans(1, 2), &trans(1, 3));
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf %lf %lf %lf", &trans(2, 0), &trans(2, 1), &trans(2, 2), &trans(2, 3));
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf %lf %lf %lf", &trans(3, 0), &trans(3, 1), &trans(3, 2), &trans(3, 3));

                data_.push_back(FramedTransformation(frame1, frame2, trans));
            }
        }
        fclose(f);
    }
}
void RGBDTrajectory::SaveToFile(std::string filename) {
    FILE * f = fopen(filename.c_str(), "w");
    for (int i = 0; i < (int)data_.size(); i++) {
        Eigen::Matrix4d & trans = data_[i].transformation_;
        fprintf(f, "%d\t%d\n", data_[i].frame1_, data_[i].frame2_);
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(0, 0), trans(0, 1), trans(0, 2), trans(0, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(3, 0), trans(3, 1), trans(3, 2), trans(3, 3));
    }
    fclose(f);
}

void RGBDTrajectory::SaveToFileAppend(std::string filename)
{
    FILE * f = fopen(filename.c_str(), "a+");
    for (int i = 0; i < (int)data_.size(); i++) {
        Eigen::Matrix4d & trans = data_[i].transformation_;
        fprintf(f, "%d\t%d\n", data_[i].frame1_, data_[i].frame2_);
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(0, 0), trans(0, 1), trans(0, 2), trans(0, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(3, 0), trans(3, 1), trans(3, 2), trans(3, 3));
    }
    fclose(f);
}

void RGBDTrajectory::SaveToSPFile(std::string filename) {
    FILE * f = fopen(filename.c_str(), "w");
    for (int i = 0; i < (int)data_.size(); i++) {
        if (data_[i].frame1_==-1)
            continue;
        Eigen::Matrix4d & trans = data_[i].transformation_;
        fprintf(f, "%d\t%d\n", data_[i].frame1_, data_[i].frame2_);
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(0, 0), trans(0, 1), trans(0, 2), trans(0, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(3, 0), trans(3, 1), trans(3, 2), trans(3, 3));
    }
    fclose(f);
}


/**
 * RGBDInformation::RGBDInformation::RGBDInformation::RGBDInformation::RGBDInformation::
 * */

void RGBDInformation::LoadFromFile(std::string filename) {
    data_.clear();
    int frame1, frame2;
    double score;
    InformationMatrix info;
    FILE * f = fopen(filename.c_str(), "r");
    if (f != NULL) {
        char buffer[1024];
        while (fgets(buffer, 1024, f) != NULL) {
            if (strlen(buffer) > 0 && buffer[0] != '#') {
                sscanf(buffer, "%d %d %lf", &frame1, &frame2,&score);
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
                data_.push_back(FramedInformation(frame1, frame2, info, score));
            }
        }
        fclose(f);
    }
}
void RGBDInformation::SaveToFile(std::string filename) {
    FILE * f = fopen(filename.c_str(), "w");
    for (int i = 0; i < (int)data_.size(); i++) {
        InformationMatrix & info = data_[i].information_;
        fprintf(f, "%d\t%d\t%.8f\n", data_[i].frame1_, data_[i].frame2_, data_[i].score_);
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(0, 0), info(0, 1), info(0, 2), info(0, 3), info(0, 4), info(0, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(1, 0), info(1, 1), info(1, 2), info(1, 3), info(1, 4), info(1, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(2, 0), info(2, 1), info(2, 2), info(2, 3), info(2, 4), info(2, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(3, 0), info(3, 1), info(3, 2), info(3, 3), info(3, 4), info(3, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(4, 0), info(4, 1), info(4, 2), info(4, 3), info(4, 4), info(4, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(5, 0), info(5, 1), info(5, 2), info(5, 3), info(5, 4), info(5, 5));
    }
    fclose(f);
}
void RGBDInformation::SaveToFileAppend(std::string filename) {
    FILE * f = fopen(filename.c_str(), "a+");
    for (int i = 0; i < (int)data_.size(); i++) {
        InformationMatrix & info = data_[i].information_;
        fprintf(f, "%d\t%d\t%.8f\n", data_[i].frame1_, data_[i].frame2_, data_[i].score_);
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(0, 0), info(0, 1), info(0, 2), info(0, 3), info(0, 4), info(0, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(1, 0), info(1, 1), info(1, 2), info(1, 3), info(1, 4), info(1, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(2, 0), info(2, 1), info(2, 2), info(2, 3), info(2, 4), info(2, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(3, 0), info(3, 1), info(3, 2), info(3, 3), info(3, 4), info(3, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(4, 0), info(4, 1), info(4, 2), info(4, 3), info(4, 4), info(4, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(5, 0), info(5, 1), info(5, 2), info(5, 3), info(5, 4), info(5, 5));
    }
    fclose(f);
}
void RGBDInformation::SaveToSPFile(std::string filename) {
    FILE * f = fopen(filename.c_str(), "w");
    for (int i = 0; i < (int)data_.size(); i++) {
        if (data_[i].frame1_==-1)
            continue;
        InformationMatrix & info = data_[i].information_;
        fprintf(f, "%d\t%d\t%.8f\n", data_[i].frame1_, data_[i].frame2_, data_[i].score_);
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(0, 0), info(0, 1), info(0, 2), info(0, 3), info(0, 4), info(0, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(1, 0), info(1, 1), info(1, 2), info(1, 3), info(1, 4), info(1, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(2, 0), info(2, 1), info(2, 2), info(2, 3), info(2, 4), info(2, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(3, 0), info(3, 1), info(3, 2), info(3, 3), info(3, 4), info(3, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(4, 0), info(4, 1), info(4, 2), info(4, 3), info(4, 4), info(4, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(5, 0), info(5, 1), info(5, 2), info(5, 3), info(5, 4), info(5, 5));
    }
    fclose(f);
}