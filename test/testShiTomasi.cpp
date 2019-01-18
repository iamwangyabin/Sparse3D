//
// Created by wang on 19-1-18.
//

#include "ShiTomasi.h"

int main(int argc, char** argv){
    std::string color_dir = argv[1];
    std::string color_correspondence_file = argv[2];

    getShiTomasi(color_dir, color_correspondence_file);

//    cv::Mat image_color = cv::imread("./color/color0.png", cv::IMREAD_COLOR);
//
//    //使用灰度图像进行角点检测
//    cv::Mat image_gray;
//    cv::cvtColor(image_color, image_gray, cv::COLOR_BGR2GRAY);
//
//    //设置角点检测参数
//    std::vector<cv::Point2f> corners;
//    int max_corners = 200;
//    double quality_level = 0.001;
//    double min_distance = 40;
//    int block_size = 3;
//    bool use_harris = false;
//    double k = 0.04;
//
//    //角点检测
//    cv::goodFeaturesToTrack(image_gray,
//                            corners,
//                            max_corners,
//                            quality_level,
//                            min_distance,
//                            cv::Mat(),
//                            block_size,
//                            use_harris,
//                            k);
//
//    //将检测到的角点绘制到原图上
//    for (int i = 0; i < corners.size(); i++)
//    {
//        cv::circle(image_color, corners[i], 1, cv::Scalar(0, 0, 255), 2, 8, 0);
//    }
//
//    cv::imshow("house corner", image_color);
//    cv::waitKey(0);
}