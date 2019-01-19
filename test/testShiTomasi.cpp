//
// Created by wang on 19-1-18.
//

#include "ShiTomasi.h"

int main(int argc, char** argv){
    std::string color_dir = argv[1];
    std::string color_correspondence_file = argv[2];

    getShiTomasi(color_dir, color_correspondence_file);

}