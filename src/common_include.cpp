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

void ImageDescriptor::LoadFromFile(std::string filename) {
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

        std::string temp_str = keyword.substr(0, 8);
        if (temp_str == "keypoint")
        {
            std::stringstream sid(keyword.substr(8, 9));
            int id;
            sid >> id;
            std::getline(input, line);
            std::stringstream sl(line);

            std::vector<float> descriptor_vec;
            float a;
            while (sl>>a)
                descriptor_vec.push_back(a);

            KeypointDescriptor data;
            data.keypoint_id = id;
            data.dvec = descriptor_vec;

            data_.push_back(data);
        }
    }

}


/**
 * RGBDTrajectory::RGBDTrajectory::RGBDTrajectory::RGBDTrajectory::RGBDTrajectory::
 * */
void RGBDTrajectory::
LoadFromFile(std::string filename) {
    data_.clear();
    int frame1, frame2;
    Eigen::Matrix4d trans;
    FILE * f = fopen(filename.c_str(), "r");
    if (f != NULL) {
        char buffer[1024];
        while (fgets(buffer, 1024, f) != NULL) {
            if (strlen(buffer) > 0 && buffer[0] != '#') {
                sscanf(buffer, "%d %d", &frame1, &frame2);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf %lf %lf %lf", &trans(0, 0), &trans(0, 1), &trans(0, 2), &trans(0, 3));
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf %lf %lf %lf", &trans(1, 0), &trans(1, 1), &trans(1, 2), &trans(1, 3));
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf %lf %lf %lf", &trans(2, 0), &trans(2, 1), &trans(2, 2), &trans(2, 3));
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf %lf %lf %lf", &trans(3, 0), &trans(3, 1), &trans(3, 2), &trans(3, 3));

                data_.push_back(FramedTransformation(frame1, frame2, trans));
            }
        }
        fclose(f);
    }
}
void RGBDTrajectory::SaveToFile(std::string filename) {
    FILE * f = fopen(filename.c_str(), "w");
    for (int i = 0; i < (int)data_.size(); i++) {
        Eigen::Matrix4d & trans = data_[i].transformation_;
        fprintf(f, "%d\t%d\n", data_[i].frame1_, data_[i].frame2_);
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(0, 0), trans(0, 1), trans(0, 2), trans(0, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(3, 0), trans(3, 1), trans(3, 2), trans(3, 3));
    }
    fclose(f);
}

void RGBDTrajectory::SaveToFileAppend(std::string filename)
{
    FILE * f = fopen(filename.c_str(), "a+");
    for (int i = 0; i < (int)data_.size(); i++) {
        Eigen::Matrix4d & trans = data_[i].transformation_;
        fprintf(f, "%d\t%d\n", data_[i].frame1_, data_[i].frame2_);
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(0, 0), trans(0, 1), trans(0, 2), trans(0, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(3, 0), trans(3, 1), trans(3, 2), trans(3, 3));
    }
    fclose(f);
}

void RGBDTrajectory::SaveToSPFile(std::string filename) {
    FILE * f = fopen(filename.c_str(), "w");
    for (int i = 0; i < (int)data_.size(); i++) {
        if (data_[i].frame1_==-1)
            continue;
        Eigen::Matrix4d & trans = data_[i].transformation_;
        fprintf(f, "%d\t%d\n", data_[i].frame1_, data_[i].frame2_);
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(0, 0), trans(0, 1), trans(0, 2), trans(0, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3));
        fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(3, 0), trans(3, 1), trans(3, 2), trans(3, 3));
    }
    fclose(f);
}


/**
 * RGBDInformation::RGBDInformation::RGBDInformation::RGBDInformation::RGBDInformation::
 * */

void RGBDInformation::LoadFromFile(std::string filename) {
    data_.clear();
    int frame1, frame2;
    double score;
    InformationMatrix info;
    FILE * f = fopen(filename.c_str(), "r");
    if (f != NULL) {
        char buffer[1024];
        while (fgets(buffer, 1024, f) != NULL) {
            if (strlen(buffer) > 0 && buffer[0] != '#') {
                sscanf(buffer, "%d %d %lf", &frame1, &frame2,&score);
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(0, 0), &info(0, 1), &info(0, 2), &info(0, 3), &info(0, 4), &info(0, 5));
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(1, 0), &info(1, 1), &info(1, 2), &info(1, 3), &info(1, 4), &info(1, 5));
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(2, 0), &info(2, 1), &info(2, 2), &info(2, 3), &info(2, 4), &info(2, 5));
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(3, 0), &info(3, 1), &info(3, 2), &info(3, 3), &info(3, 4), &info(3, 5));
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(4, 0), &info(4, 1), &info(4, 2), &info(4, 3), &info(4, 4), &info(4, 5));
                fgets(buffer, 1024, f);
                sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(5, 0), &info(5, 1), &info(5, 2), &info(5, 3), &info(5, 4), &info(5, 5));
                data_.push_back(FramedInformation(frame1, frame2, info, score));
            }
        }
        fclose(f);
    }
}
void RGBDInformation::SaveToFile(std::string filename) {
    FILE * f = fopen(filename.c_str(), "w");
    for (int i = 0; i < (int)data_.size(); i++) {
        InformationMatrix & info = data_[i].information_;
        fprintf(f, "%d\t%d\t%.8f\n", data_[i].frame1_, data_[i].frame2_, data_[i].score_);
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(0, 0), info(0, 1), info(0, 2), info(0, 3), info(0, 4), info(0, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(1, 0), info(1, 1), info(1, 2), info(1, 3), info(1, 4), info(1, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(2, 0), info(2, 1), info(2, 2), info(2, 3), info(2, 4), info(2, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(3, 0), info(3, 1), info(3, 2), info(3, 3), info(3, 4), info(3, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(4, 0), info(4, 1), info(4, 2), info(4, 3), info(4, 4), info(4, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(5, 0), info(5, 1), info(5, 2), info(5, 3), info(5, 4), info(5, 5));
    }
    fclose(f);
}
void RGBDInformation::SaveToFileAppend(std::string filename) {
    FILE * f = fopen(filename.c_str(), "a+");
    for (int i = 0; i < (int)data_.size(); i++) {
        InformationMatrix & info = data_[i].information_;
        fprintf(f, "%d\t%d\t%.8f\n", data_[i].frame1_, data_[i].frame2_, data_[i].score_);
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(0, 0), info(0, 1), info(0, 2), info(0, 3), info(0, 4), info(0, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(1, 0), info(1, 1), info(1, 2), info(1, 3), info(1, 4), info(1, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(2, 0), info(2, 1), info(2, 2), info(2, 3), info(2, 4), info(2, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(3, 0), info(3, 1), info(3, 2), info(3, 3), info(3, 4), info(3, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(4, 0), info(4, 1), info(4, 2), info(4, 3), info(4, 4), info(4, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(5, 0), info(5, 1), info(5, 2), info(5, 3), info(5, 4), info(5, 5));
    }
    fclose(f);
}
void RGBDInformation::SaveToSPFile(std::string filename) {
    FILE * f = fopen(filename.c_str(), "w");
    for (int i = 0; i < (int)data_.size(); i++) {
        if (data_[i].frame1_==-1)
            continue;
        InformationMatrix & info = data_[i].information_;
        fprintf(f, "%d\t%d\t%.8f\n", data_[i].frame1_, data_[i].frame2_, data_[i].score_);
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(0, 0), info(0, 1), info(0, 2), info(0, 3), info(0, 4), info(0, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(1, 0), info(1, 1), info(1, 2), info(1, 3), info(1, 4), info(1, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(2, 0), info(2, 1), info(2, 2), info(2, 3), info(2, 4), info(2, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(3, 0), info(3, 1), info(3, 2), info(3, 3), info(3, 4), info(3, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(4, 0), info(4, 1), info(4, 2), info(4, 3), info(4, 4), info(4, 5));
        fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(5, 0), info(5, 1), info(5, 2), info(5, 3), info(5, 4), info(5, 5));
    }
    fclose(f);
}

template<typename T>
bool SeqSaveAndLoad::Load(std::string filename, std::vector<T>& seq)
{
    std::ifstream input(filename);
    if (!input.good())
        return false;
    while (input.good()){
        std::string str;
        input >> str;
        if (str != ""&&str != "\n")
        {
            std::stringstream ss(str);
            T l;
            ss >> l;
            seq.push_back(l);
        }
    }

    return true;
}


template<typename T>
bool SeqSaveAndLoad::Save(std::string filename, std::vector<T>& seq)
{
    std::ofstream output(filename);
    for (int i = 0; i < seq.size(); ++i)
    {
        if (i < seq.size() - 1)
            output << seq[i] << "\n";
        else
            output << seq[i];
    }

    return true;
}

template<typename T>
bool SeqSaveAndLoad::Save(std::string filename, std::vector<T>& seq, RGBDTrajectory& loop)
{
    std::ofstream output(filename);
    for (int i = 0; i < seq.size(); ++i)
    {
        if (i < seq.size() - 1)
            output << loop.data_[i].frame1_ << "---" << loop.data_[i].frame2_ << ":  " << seq[i] << "\n";
        else
            output << loop.data_[i].frame1_ << "---" << loop.data_[i].frame2_ << ":  " << seq[i];
    }
    return true;
}