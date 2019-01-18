//
// Created by wang on 19-1-15.
//
#include "Narf.h"

int getNarf(std::string pointcloud_xyzn_dir, std::string keypoint_dir, std::string descriptor_dir) {

    if (!boost::filesystem::exists(keypoint_dir))
        boost::filesystem::create_directory(keypoint_dir);
    if (!boost::filesystem::exists(descriptor_dir))
        boost::filesystem::create_directory(descriptor_dir);


    int num_of_pcd = static_cast<int>(std::count_if(
            boost::filesystem::directory_iterator(boost::filesystem::path(pointcloud_xyzn_dir)),
            boost::filesystem::directory_iterator(),
            [](const boost::filesystem::directory_entry &e) {
                return e.path().extension() == ".pcd";}));

    for (int i = 0; i < num_of_pcd; ++i) {
        // normal point cloud
        std::stringstream nss;
        nss << pointcloud_xyzn_dir << "pointcloud_xyzn" << i << ".pcd";
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scene_xyzn(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::io::loadPCDFile(nss.str(), *scene_xyzn);
        pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);

        for (int t = 0; t < scene_xyzn->points.size(); ++t)
        {
            /*brief A point structure representing normal coordinates and the surface curvature estimate. (SSE friendly)ingroup common*/
            pcl::Normal n;
            n.normal_x = scene_xyzn->points[t].normal_x;
            n.normal_y = scene_xyzn->points[t].normal_y;
            n.normal_z = scene_xyzn->points[t].normal_z;
            normal->push_back(n);
        }

        // point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto &point : scene_xyzn->points) {
            pcl::PointXYZ p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            scene_xyz->push_back(p);
        }

        // keypoint
        std::cout << "Detect pointcloud" << i << " keypoint\n";

        NARF narf(scene_xyz);
        narf.narf_keypoints_extraction();
        std::cout<<narf.keypoints.size() <<std::endl;
        if (narf.keypoints.size() <= 0)
            continue;

        std::cout<<i<<std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(narf.keypoints, *pointcloud_keypoints);
        pointcloud_keypoints->width = pointcloud_keypoints->size();
        pointcloud_keypoints->height = 1;

        // descriptor
        std::cout << "Detect pointcloud" << i << " descriptor\n";
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptor(new pcl::PointCloud<pcl::FPFHSignature33>());

        pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud(pointcloud_keypoints);
        fpfh.setSearchSurface(scene_xyz);
        fpfh.setInputNormals(normal);
        fpfh.setRadiusSearch(0.06);
        fpfh.compute(*descriptor);
        // save
        if (pointcloud_keypoints->points.size() > 100){
            pointcloud_keypoints->points.resize(100);
            descriptor->points.resize(100);
            pointcloud_keypoints->width = pointcloud_keypoints->points.size();
        }
        std::stringstream kss;
        kss << keypoint_dir << "keypoints" << i << ".pcd";
        pcl::io::savePCDFile(kss.str(), *pointcloud_keypoints);

        std::stringstream dss;
        dss << descriptor_dir << "descriptor" << i << ".txt";
        std::ofstream d_file(dss.str());
        d_file << "Descriptor vector (FPFHSignature33):\n";
        for (int k = 0; k < descriptor->points.size(); ++k)
        {
            d_file << "keypoint" << k << "\n";
            for (int t = 0; t < 33; ++t)
                d_file << descriptor->points[k].histogram[t] << " ";

            d_file << "\n";
        }

    }
    return 0;
}

NARF::NARF(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pointer){
    point_cloud_ptr = point_cloud_pointer;
    pcl::PointCloud<pcl::PointXYZ>& point_cloud = *point_cloud_pointer;

    float angular_resolution = 0.2f;
    angular_resolution = pcl::deg2rad(angular_resolution);

    float maxAngleWidth = (float)(180.0 * (M_PI / 180.0f));
    float maxAngleHeight = (float)(180.0 * (M_PI / 180.0f));

    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());

    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

    float noise_level = 0.0f;
    float min_range = 0.0f;
    int border_size = 0.05;

    range_image.createFromPointCloud(point_cloud, angular_resolution, maxAngleWidth, maxAngleHeight, scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
}

void NARF::narf_keypoints_extraction(){
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector;
    narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage(&range_image);
    narf_keypoint_detector.getParameters().support_size = support_size;

    keypoint_indices.clear();
    narf_keypoint_detector.compute(keypoint_indices);

    std::cout << "Found " << keypoint_indices.points.size() << " key points.\n";

    keypoints.points.resize(keypoint_indices.points.size());
    for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
        keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
}

