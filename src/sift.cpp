//
// Created by wang on 19-1-14.
//

///usr/local/include/opencv2/xfeatures2d/nonfree.hpp

#include "sift.h"

using namespace cv;
using namespace std;
using namespace xfeatures2d;

int sift(string input,string output){
    string color_dir, color_correspondence_file;
    color_correspondence_file=output;
    color_dir=input;

    if (boost::filesystem::exists(color_correspondence_file))
        boost::filesystem::remove(color_correspondence_file);

    /** directory_iterator(p)就是迭代器的起点，无参数的directory_iterator()就是迭代器的终点*/
    int num_of_color;
    num_of_color = static_cast<int>(std::count_if(
                        boost::filesystem::directory_iterator(boost::filesystem::path(color_dir)),
                        boost::filesystem::directory_iterator(),
                        [](const boost::filesystem::directory_entry& e) {
                            return e.path().extension() == ".png";  }));

    vector<vector<KeyPoint>> img_keypoints(num_of_color);		// keypoints array
    vector<Mat> img_descriptors(num_of_color);					// descriptors array

//    Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();
    cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> sift_detector = cv::xfeatures2d::SiftFeatureDetector::create();

//    SiftFeatureDetector feature;
//    xfeatures2d::SiftDescriptorExtractor descript;
#pragma omp parallel for num_threads( 8 ) schedule( dynamic )
    for (int i = 0; i < num_of_color; ++i) {
        stringstream ss1;
        ss1 << color_dir << "color" << i << ".png";
        Mat image1 = imread(ss1.str(), 0);
        vector<KeyPoint> keypoints1;
        sift_detector->detect(image1, keypoints1);
        // save
        img_keypoints[i] = keypoints1;
        Mat description1;
        sift_detector->compute(image1, img_keypoints[i], description1);
        img_descriptors[i] = description1;
    }

    // correspondence
    struct trip
    {
        int pixel1_x, pixel1_y;
        int pixel2_x, pixel2_y;
    };
    struct color_match
    {
        int img1, img2;
        vector<trip> cor_pixel;
    };

    ofstream ouput_file(color_correspondence_file);
    ouput_file << "# Save correspondence pixel in color image.\n# Format: \n# image_id image_i image_j \n# pixel_i_x pixel_i_y pixel_j_x pixel_j_y\n\n";

#pragma omp parallel for num_threads( 8 ) schedule( dynamic )
    for (int i = 0; i < num_of_color; ++i)
    {
        vector<color_match> good_matches;

        for (int j = i + 1; j < num_of_color; ++j) {
            vector<KeyPoint> &keypoints1 = img_keypoints[i];
            vector<KeyPoint> &keypoints2 = img_keypoints[j];
            Mat &description1 = img_descriptors[i];
            Mat &description2 = img_descriptors[j];

            if (keypoints1.size() <= 0 || keypoints2.size() <= 0)
                continue;

            //kd-tree
            flann::Index kdtree(description2, flann::KDTreeIndexParams(4));
            Mat m_indices(description1.rows, 2, CV_32S);
            Mat m_dists(description1.rows, 2, CV_32F);
            kdtree.knnSearch(description1, m_indices, m_dists, 2, cv::flann::SearchParams(64));

            // result
            color_match mat;
            mat.img1 = i;
            mat.img2 = j;

            for (int t = 0; t < description1.rows; ++t) {
                if (m_dists.at<float>(t, 1) * 0.4 > m_dists.at<float>(t, 0)) {
                    int x1 = cvRound(keypoints1[t].pt.x);
                    int y1 = cvRound(keypoints1[t].pt.y);

                    int x2 = cvRound(keypoints2[m_indices.at<int>(t, 0)].pt.x);
                    int y2 = cvRound(keypoints2[m_indices.at<int>(t, 0)].pt.y);

                    trip tri;
                    tri.pixel1_x = x1;
                    tri.pixel1_y = y1;
                    tri.pixel2_x = x2;
                    tri.pixel2_y = y2;
                    mat.cor_pixel.push_back(tri);
                }
            }

            good_matches.push_back(mat);
        }

#pragma omp critical
        {
            for (int t = 0; t < good_matches.size(); ++t)
            {
                if (good_matches[t].cor_pixel.size() >= 4)
                {
                    ouput_file << "image_id " << good_matches[t].img1 << " " << good_matches[t].img2 << "\n";
                    for (int k = 0; k < good_matches[t].cor_pixel.size(); ++k)
                    {
                        if (k > 100)
                            break;
                        ouput_file << good_matches[t].cor_pixel[k].pixel1_x << " "
                                   << good_matches[t].cor_pixel[k].pixel1_y << " "
                                   << good_matches[t].cor_pixel[k].pixel2_x << " "
                                   << good_matches[t].cor_pixel[k].pixel2_y << "\n";
                    }
                }
            }
        }
    }
    ouput_file.close();
    return 0;
}
