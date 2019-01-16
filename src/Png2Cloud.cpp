//
// Created by wang on 19-1-15.
//
/**
 * 这个是生成云图的程序
 * 输入就是rgb图片和depth图片，然后输出一组对应的点云程序
 * */


#include "Png2Cloud.h"
#include <pcl/io/pcd_io.h>

int ChangePng2Cloud(    std::string depth_dir ,
                  std::string color_dir ,
                  std::string camera_para ,
                  std::string pointcloud_dir ,
                  std::string pointcloud_ds_dir ,
                  std::string xyzn_dir ,
                  std::string xyzn_ds_dir){
    if (!boost::filesystem::exists(pointcloud_dir))
        boost::filesystem::create_directory(pointcloud_dir);
    if (!boost::filesystem::exists(pointcloud_ds_dir))
        boost::filesystem::create_directory(pointcloud_ds_dir);
    if (!boost::filesystem::exists(xyzn_dir))
        boost::filesystem::create_directory(xyzn_dir);
    if (!boost::filesystem::exists(xyzn_ds_dir))
        boost::filesystem::create_directory(xyzn_ds_dir);

    int num_of_depth;
    num_of_depth = static_cast<int>(std::count_if(
                    boost::filesystem::directory_iterator(boost::filesystem::path(depth_dir)),
                    boost::filesystem::directory_iterator(),
                    [](const boost::filesystem::directory_entry& e) {
                        return e.path().extension() == ".png";  }));

    int num_of_color;
    num_of_color = static_cast<int>(std::count_if(
                    boost::filesystem::directory_iterator(boost::filesystem::path(color_dir)),
                    boost::filesystem::directory_iterator(),
                    [](const boost::filesystem::directory_entry& e) {
                        return e.path().extension() == ".png";  }));

    Png2Cloud app;
    app._camera.LoadFromFile(camera_para);

    for (int i = 0; i < num_of_color;++i)
    {
        // load
        cv::Mat color;
        cv::Mat depth;
        app.Load(i, depth_dir, color_dir, depth, color);

        // Png to RGBcloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = app.Png2RGBcloud(depth, color);
        std::stringstream ss1;
        ss1 << pointcloud_dir << "pointcloud" << i << ".pcd";
        pcl::io::savePCDFile(ss1.str(), *cloud, true);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ds = app.DownSamle(cloud);
        std::stringstream ss2;
        ss2 << pointcloud_ds_dir << "pointcloud_ds" << i << ".pcd";
        pcl::io::savePCDFile(ss2.str(), *cloud_ds, true);

        // compute normal
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr raw_cloud_n = app.ComputeModelNormal(cloud);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_n(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        for (int j = 0; j < raw_cloud_n->points.size(); j++) {
            if (!std::isnan(raw_cloud_n->points[j].normal_x)) {
                cloud_n->push_back(raw_cloud_n->points[j]);
            }
        }
        cloud_n->width = cloud_n->points.size();
        cloud_n->height = 1;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_n_ds = app.DownSamle(cloud_n);

        std::stringstream ss3;
        ss3 << xyzn_dir << "pointcloud_xyzn" << i << ".pcd";
        pcl::io::savePCDFile(ss3.str(), *cloud_n, true);

        std::stringstream ss4;
        ss4 << xyzn_ds_dir << "pointcloud_ds_xyzn" << i << ".pcd";
        pcl::io::savePCDFile(ss4.str(), *cloud_n_ds, true);
    }
    return 0;
}




bool Png2Cloud::Load(int png_index, std::string& depth_dir, std::string& color_dir, cv::Mat& depth_img, cv::Mat& color_img){
    std::stringstream ss;
    ss << depth_dir  <<"depth"<< png_index << ".png";
    std::string filepath_depth = ss.str();

    std::stringstream sss;
    sss << color_dir  <<"color"<< png_index << ".png";
    std::string filepath_color = sss.str();

    depth_img = cv::imread(filepath_depth, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
    depth_img.convertTo(depth_img, CV_16U);

    assert(_camera.img_width_ == depth_img.cols && _camera.img_height_ == depth_img.rows);

    color_img = cv::imread(filepath_color, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
    color_img.convertTo(color_img, CV_8UC3);

    return true;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Png2Cloud::Png2RGBcloud(cv::Mat& depth_img, cv::Mat& color_img)
{
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < depth_img.cols; ++i)		//i => u
    {
        for (int j = 0; j < depth_img.rows; ++j)	//j => v
        {
            double x, y, z;
            if (UVD2XYZ(i, j, depth_img.at<ushort>(j, i), x, y, z))
            {
                // color pointcloud
                pcl::PointXYZRGB rgb;
                rgb.x = x;	rgb.y = y;	rgb.z = z;

                cv::Vec3b intensity = color_img.at<cv::Vec3b>(j, i);
                rgb.b = intensity.val[0];
                rgb.g = intensity.val[1];
                rgb.r = intensity.val[2];
                // 把p加入到点云中
                cloud->points.push_back(rgb);
            }

        }
    }
    return cloud;
}

void Png2Cloud::ConvertRGBCloud2XYZCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr rgb, pcl::PointCloud<pcl::PointXYZ>::Ptr xyz)
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

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Png2Cloud::ComputeModelNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, float radius)
{
    radius = static_cast<float>(_camera.downsample_leaf);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // convert rgb cloud to xyz cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    ConvertRGBCloud2XYZCloud(scene, scene_xyz);


    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setRadiusSearch(radius);
    norm_est.setInputCloud(scene_xyz);
    norm_est.compute(*normals);

    assert(normals->points.size() == scene->points.size());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    for (int i = 0; i < scene->points.size(); ++i)
    {
        pcl::PointXYZRGBNormal p;
        p.x = scene->points[i].x;
        p.y = scene->points[i].y;
        p.z = scene->points[i].z;
        p.r = scene->points[i].r;
        p.g = scene->points[i].g;
        p.b = scene->points[i].b;
        p.normal_x = normals->points[i].normal_x;
        p.normal_y = normals->points[i].normal_y;
        p.normal_z = normals->points[i].normal_z;
        result->points.push_back(p);
    }
    return result;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Png2Cloud::DownSamle(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_in)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    float leaf = _camera.downsample_leaf;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(cloud_in);
    grid.filter(*cloud_out);
    return cloud_out;
}


pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Png2Cloud::DownSamle(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr cloud_in)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
    float leaf = _camera.downsample_leaf;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(cloud_in);
    grid.filter(*cloud_out);
    return cloud_out;
}

bool Png2Cloud::UVD2XYZ(int u, int v, unsigned short d, double & x, double & y, double & z) {
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

bool Png2Cloud::XYZ2UVD(double x, double y, double z, int & u, int & v, unsigned short & d) {
    if (z > 0) {
        float x_ = x / z;	float y_ = y / z;
        float r2 = x_*x_ + y_*y_;
        float x__ = x_*(1 + _camera.k1_*r2 + _camera.k2_*r2*r2 + _camera.k3_*r2*r2*r2) + 2 * _camera.p1_*x_*y_ + _camera.p2_*(r2 + 2 * x_*x_);
        float y__ = y_*(1 + _camera.k1_*r2 + _camera.k2_*r2*r2 + _camera.k3_*r2*r2*r2) + _camera.p1_*(r2 + 2 * y_*y_) + 2 * _camera.p2_*x_*y_;
        float uu = x__ * _camera.fx_ + _camera.cx_;
        float vv = y__ * _camera.fy_ + _camera.cy_;
        u = static_cast<int>(round(uu));
        v = static_cast<int>(round(vv));
        d = static_cast<unsigned short>(round(z * _camera.depth_ratio_));

        // ideal model
        /*u = round(x * _camera.fx_ / z + _camera.cx_);
        v = round(y * _camera.fy_ / z + _camera.cy_);
        d = static_cast<unsigned short>(round(z * _camera.depth_ratio_));*/
        return (u >= 0 && u < _camera.img_width_ && v >= 0 && v < _camera.img_height_);
    }
    else {
        return false;
    }
}