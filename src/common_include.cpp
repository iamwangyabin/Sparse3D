//
// Created by wang on 19-1-16.
//

#include "common_include.h"


void CameraParam::LoadFromFile(std::string filename) {
    FILE * f = fopen(filename.c_str(), "r");
    if (f != NULL) {
        char buffer[1024];
        while (fgets(buffer, 1024, f) != NULL) {
            if (strlen(buffer) > 0 && buffer[0] != '#') {
                sscanf(buffer, "%lf", &fx_);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf", &fy_);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf", &cx_);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf", &cy_);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf", &k1_);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf", &k2_);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf", &k3_);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf", &p1_);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf", &p2_);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf", &depth_ratio_);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf", &downsample_leaf);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%d", &img_width_);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%d", &img_height_);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf", &integration_trunc_);
            }
        }
        printf("Camera model set to (fx: %.2f, fy: %.2f, cx: %.2f, cy: %.2f, k1: %.2f, k2: %.2f, k3: %.2f, p1: %.2f, p2: %.2f, depth_ratio_: %.2f, downsample_leaf: %.2f, img_width_: %d, img_height_: %d, integration_trunc: %.2f)\n",
               fx_, fy_, cx_, cy_, k1_, k2_, k3_, p1_, p2_, depth_ratio_, downsample_leaf, img_width_, img_height_, integration_trunc_);
        fclose(f);
    }
}
