//
// Created by wang on 19-1-16.
//
#include "ColorCorrespondence.h"


int ColorCorrespondence(std::string pointcloud_dir, std::string pointcloud_ds_dir, std::string depth_dir, std::string correspendence_file, std::string camera_file,double score_max_depth, std::string traj_file, std::string info_file)
{
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

        if (graph_corps->size() < 4)
            continue;
        // 得到变换矩阵方法，先构造单位矩阵，再乘出来
        Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
        transformation_matrix = transformation_matrix*gm._transformation;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene1_ds = downsample_pc[img1];
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene2_ds = downsample_pc[img2];

        BuildCorpPointSet buidCorp;
        int K_set = buidCorp.ComputeCorrepondencePointSet(transformation_matrix, scene1_ds, scene2_ds, score_max_depth);
        /**
         * overlap ratio > 30% will be a potential alignment between Fi and Fj
         * */
        if (K_set < buidCorp._total_size / 3 || K_set == 0 || buidCorp._total_size == 0) {
            continue;
        }

        Eigen::Matrix<double ,6,6> info_mat = buidCorp.ComputeInfoMatrix(buidCorp._correspondences,scene1_ds,scene2_ds);
        double score = (double)K_set / (double)buidCorp._total_size;
        InformationMatrix im = info_mat;
        FramedInformation fi(img1, img2, im, score);
        info.data_[img1*num_of_pc + img2] = fi;

        Eigen::Matrix4d tf = transformation_matrix.cast<double>();
        FramedTransformation ft(img1, img2, tf);
        traj.data_[img1*num_of_pc + img2] = ft;
    }
    //save
    traj.SaveToSPFile(traj_file);
    info.SaveToSPFile(info_file);

    return 0;
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



pcl::PointXYZRGB  SearchNearestValidPoint(int u, int v, cv::Mat& depth, CameraParam& camera){
    int uu = u, vv = v;
    ushort z1 = depth.at<ushort>(vv, uu);
    double x, y, z;
    int step = 1;
    int forward8 = 0;
    while (!UVD2XYZ(uu, vv, z1, camera, x, y, z) && step<100)
    {
        uu = u;
        vv = v;
        forward8++;
        if (forward8 == 1)
        {
            uu += step;
            if (uu >= depth.cols)
                uu -= step;
            z1 = depth.at<ushort>(vv, uu);
            continue;
        }
        if (forward8 == 2)
        {
            vv -= step;
            if (vv < 0)
                vv += step;
            z1 = depth.at<ushort>(vv, uu);
            continue;
        }
        if (forward8 == 3)
        {
            uu -= step;
            if (uu < 0)
                uu += step;
            z1 = depth.at<ushort>(vv, uu);
            continue;
        }
        if (forward8 == 4)
        {
            vv += step;
            if (vv >= depth.rows)
                vv -= step;
            z1 = depth.at<ushort>(vv, uu);
            continue;
        }
        if (forward8 == 5)
        {
            uu += step;
            vv -= step;
            if (vv < 0 || uu >= depth.cols)
            {
                uu -= step;
                vv += step;
            }
            z1 = depth.at<ushort>(vv, uu);
            continue;
        }
        if (forward8 == 6)
        {
            uu -= step;
            vv -= step;
            if (vv <0  || uu <0)
            {
                uu += step;
                vv += step;
            }
            z1 = depth.at<ushort>(vv, uu);
            continue;
        }
        if (forward8 == 7)
        {
            uu -= step;
            vv += step;
            if (vv >= depth.rows || uu < 0)
            {
                uu += step;
                vv -= step;
            }
            z1 = depth.at<ushort>(vv, uu);
            continue;
        }
        if (forward8 == 8)
        {
            uu += step;
            vv += step;
            if (vv >= depth.rows || uu >= depth.cols)
            {
                uu -= step;
                vv -= step;
            }
            z1 = depth.at<ushort>(vv, uu);
            continue;
        }
        step++;
        forward8 = 0;
    }
    pcl::PointXYZRGB p;
    if (z1==0)
    {
        p.x = x;	p.y = y;	p.z = -1.0f;
        p.r = 255;	p.g = 255;	p.b = 255;
    }
    else
    {
        p.x = x;	p.y = y;	p.z = z;
        p.r = 255;	p.g = 255;	p.b = 255;
    }

    return p;
}


void ConvertRGBCloud2XYZCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr rgb, pcl::PointCloud<pcl::PointXYZ>::Ptr xyz)
{
    for (int i = 0; i < rgb->points.size(); ++i)
    {
        float x = rgb->points[i].x;
        float y = rgb->points[i].y;
        float z = rgb->points[i].z;
        pcl::PointXYZ p;
        p.x = x;	p.y = y;	p.z = z;
        xyz->points.push_back(p);
    }
}

void ConvertXYZCloud2RGBCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr xyz, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb)
{
    for (int i = 0; i < xyz->points.size(); ++i)
    {
        float x = xyz->points[i].x;
        float y = xyz->points[i].y;
        float z = xyz->points[i].z;
        pcl::PointXYZRGB p;
        p.x = x;	p.y = y;	p.z = z;
        p.r = 255;	p.g = 255;	p.b = 255;
        rgb->points.push_back(p);
    }
}

bool UVD2XYZ(int u, int v, unsigned short d, CameraParam& _camera, double& x, double& y, double& z)
{
    if (d > 0) {
        cv::Mat cam(3, 3, CV_32F);
        cam.at<float>(0, 0) = _camera.fx_; cam.at<float>(0, 1) = 0; cam.at<float>(0, 2) = _camera.cx_;
        cam.at<float>(1, 0) = 0; cam.at<float>(1, 1) = _camera.fy_; cam.at<float>(1, 2) = _camera.cy_;
        cam.at<float>(2, 0) = 0; cam.at<float>(2, 1) = 0; cam.at<float>(2, 2) = 1;
        cv::Mat dist_coef(1, 5, CV_32F);
        dist_coef.at<float>(0, 0) = _camera.k1_;
        dist_coef.at<float>(0, 1) = _camera.k2_;
        dist_coef.at<float>(0, 2) = _camera.p1_;
        dist_coef.at<float>(0, 3) = _camera.p2_;
        dist_coef.at<float>(0, 4) = _camera.k3_;

        cv::Mat mat(1, 2, CV_32F);
        mat.at<float>(0, 0) = u;
        mat.at<float>(0, 1) = v;
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, cam, dist_coef, cv::Mat(), cam);
        mat = mat.reshape(1);

        float uu = mat.at<float>(0, 0);
        float vv = mat.at<float>(0, 1);

        z = d / _camera.depth_ratio_;
        x = (uu - _camera.cx_) * z / _camera.fx_;
        y = (vv - _camera.cy_) * z / _camera.fy_;


        // ideal model
        /*z = d / _camera.depth_ratio_;
        x = (u - _camera.cx_) * z / _camera.fx_;
        y = (v - _camera.cy_) * z / _camera.fy_;*/
        return true;
    }
    else {
        return false;
    }
}
