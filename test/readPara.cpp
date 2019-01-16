//
// Created by wang on 19-1-16.
//
/**
 *  测试一下读取相机参数
 * */

#include "Png2Cloud.h"
#include "iostream"

int main(int argc, char** argv){
    std::string input;
    input = argv[1];

    CameraParam app;
    app.LoadFromFile(input);
}

