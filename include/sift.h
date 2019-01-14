//
// Created by wang on 19-1-14.
//

#ifndef SPARSE3D_SIFT_H
#define SPARSE3D_SIFT_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
//#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <set>

using namespace std;

int sift(string input,string output);

#endif //SPARSE3D_SIFT_H
