//
// Created by wang on 19-2-22.
//

#include "jointPointCloud.h"

int showpcd(std::string filename){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    pcl::visualization::CloudViewer viewer("Viewer");
    viewer.showCloud(cloud);

    while( !viewer.wasStopped() )
    {

    }
    return (0);
}


void RGBDTrajectory::LoadFromFile(std::string filename){
    data_.clear();
    int frame, frame_;
    Eigen::Matrix4d trans;
    ifstream infile(filename);

    while (infile.peek()!=EOF){
//        cout<<"1"<<endl;
        infile >> frame >> frame_;
        infile >> trans(0, 0) >> trans(0, 1) >> trans(0, 2) >> trans(0, 3);
        infile >> trans(1, 0) >> trans(1, 1) >> trans(1, 2) >> trans(1, 3);
        infile >> trans(2, 0) >> trans(2, 1) >> trans(2, 2) >> trans(2, 3);
        infile >> trans(3, 0) >> trans(3, 1) >> trans(3, 2) >> trans(3, 3);
        data_.emplace_back(frame, trans);

//        cout<<frame<<endl;
//        cout<< trans(0, 0)<<" "<<trans(0, 1)<<" "<<trans(0, 2)<<" "<<trans(0, 3)<<endl;
//        cout<< trans(1, 0)<<" "<<trans(1, 1)<<" "<<trans(1, 2)<<" "<<trans(1, 3)<<endl;
//        cout<< trans(2, 0)<<" "<<trans(2, 1)<<" "<<trans(2, 2)<<" "<<trans(2, 3)<<endl;
//        cout<< trans(3, 0)<<" "<<trans(3, 1)<<" "<<trans(3, 2)<<" "<<trans(3, 3)<<endl;
//        cout<<endl;
    }
}

void joinPointCloud(std::string pos_file,std::string pointcloud_dir){

    int num_of_pc = static_cast<int>(std::count_if(
            boost::filesystem::directory_iterator(boost::filesystem::path(pointcloud_dir)),
            boost::filesystem::directory_iterator(),
            [](const boost::filesystem::directory_entry& e) {
                return e.path().extension() == ".pcd";  }));

    // Load all downsample pointcloud
//    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointclouds;
//    for (int i = 0; i < num_of_pc;++i)
//    {
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>);
//
//        std::stringstream dsss1;
//        dsss1 << pointcloud_dir << "pointcloud" << i << ".pcd";
//        pcl::io::loadPCDFile(dsss1.str(), *scene);
//        pointclouds.push_back(scene);
//    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    RGBDTrajectory traj;
    traj.LoadFromFile(pos_file);

//    if (num_of_pc==traj.data_.size())
    for (int i = 0; i < num_of_pc;++i)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::stringstream dsss1;
        dsss1 << pointcloud_dir << "pointcloud" << i << ".pcd";
        pcl::io::loadPCDFile(dsss1.str(), *scene);
        FramedTransformation t = traj.data_[i];
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transed(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud( *scene, *transed, t.transformation_ );
        *newCloud += *transed;
        i=i+10;
    }

    pcl::visualization::CloudViewer viewer("Viewer");
    viewer.showCloud(newCloud);

    while( !viewer.wasStopped() )
    {

    }

}
