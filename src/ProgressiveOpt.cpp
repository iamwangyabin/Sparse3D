//
// Created by wang on 19-1-24.
//

#include <ProgressiveOpt.h>

#include "ProgressiveOpt.h"










/**
 * Beam Search（集束搜索）是一种启发式图搜索算法
 * 在图的解空间比较大的情况下，
 * 在每一步深度扩展的时候，剪掉一些质量比较差的结点，保留下一些质量较高的结点。
 *
 * */
std::vector<int> ProgressiveOpt::BeamSearch(const double err_threshold, const int beam_width,
                                            std::map<key, std::vector<int>, key_comp> &index_map,
                                            const std::vector<key> &loop, const std::vector<int> &seq) {







    return std::vector<int>();
}

void ProgressiveOpt::LoopError(
        const std::vector<std::vector<inx2seq>> &all_loop,
        int loop_len,
        /*output parameter*/
        double &min_error,
        std::vector<inx2seq> &best_loop) {
    min_error = 99999.0;
    best_loop.clear();

    for (int i = 0; i < all_loop.size(); ++i) {
        const std::vector<inx2seq> &one_loop = all_loop[i];

        RGBDTrajectory t_arr;
        RGBDInformation2 i_arr;
        Eigen::Matrix4d err = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d final_mat = Eigen::Matrix4d::Identity();
        for (int k = 0; k < one_loop.size(); k++) {
            if (one_loop[k].seq < 1)
                continue;
            FramedTransformation &t = traj_.data_[one_loop[k].index];
            t_arr.data_.push_back(t);

            FrameInformation &inf = info_.data_[one_loop[k].index];
            i_arr.data_.push_back(inf);
        }

        double rotation_deg = -9999.0;
        int low_score_edge = 0;
        int geometric_num = 0;
        for (int i = 0; i < t_arr.data_.size(); ++i) {
            if (i == 1) {
                final_mat = t_arr.data_[i].transformation_;
                if (i_arr.data_[i].flag == 0)
                    geometric_num++;
                if (i_arr.data_[i].flag == 0 && i_arr.data_[i].score_ < low_score_threshold_)
                    low_score_edge++;

                Eigen::Matrix3d mat3;
                mat3 <<
                     t_arr.data_[i].transformation_(0, 0), t_arr.data_[i].transformation_(0,
                                                                                          1), t_arr.data_[i].transformation_(
                        0, 2),
                        t_arr.data_[i].transformation_(1, 0), t_arr.data_[i].transformation_(1,
                                                                                             1), t_arr.data_[i].transformation_(
                        1, 2),
                        t_arr.data_[i].transformation_(2, 0), t_arr.data_[i].transformation_(2,
                                                                                             1), t_arr.data_[i].transformation_(
                        2, 2);
                Eigen::Quaterniond q(mat3);
                /**将acos返回的弧度值转换为角度值
                 * 计算的是旋转的度数
                 * */
                double deg = acos(q.w()) * 180 / 3.1415;
                deg *= 2;
                if (rotation_deg < deg)
                    rotation_deg = deg;
            } else {
                err = err * t_arr.data_[i].transformation_;
                if (i_arr.data_[i].flag == 0)
                    geometric_num++;
                if (i_arr.data_[i].flag == 0 && i_arr.data_[i].score_ < low_score_threshold_)
                    low_score_edge++;

                Eigen::Matrix3d mat3;
                mat3 <<
                     t_arr.data_[i].transformation_(0, 0), t_arr.data_[i].transformation_(0,
                                                                                          1), t_arr.data_[i].transformation_(
                        0, 2),
                        t_arr.data_[i].transformation_(1, 0), t_arr.data_[i].transformation_(1,
                                                                                             1), t_arr.data_[i].transformation_(
                        1, 2),
                        t_arr.data_[i].transformation_(2, 0), t_arr.data_[i].transformation_(2,
                                                                                             1), t_arr.data_[i].transformation_(
                        2, 2);
                Eigen::Quaterniond q(mat3);
                double deg = acos(q.w()) * 180 / 3.1415;
                deg *= 2;
                if (rotation_deg < deg)
                    rotation_deg = deg;
            }
        }
        /***888888888888888888888888888888888888888888888*/
    }
}

bool ProgressiveOpt::Init() {
    namespace fs = boost::filesystem;
    namespace fs = boost::filesystem;
    if (fs::exists(fs::path(traj_file_))) {
        traj_.LoadFromFile(traj_file_);
        if (fs::exists(fs::path(info_file_))) {
            info_.LoadFromFile(info_file_);
        }
    }

    for (auto &i : traj_.data_) {
        Eigen::Matrix4d temp = i.transformation_.inverse();
        i.transformation_ = temp;
    }

    // all pose init to identity matrix
    for (int i = 0; i < frame_num_; ++i) {
        Eigen::Matrix4d mat4d_I = Eigen::Matrix4d::Identity();
        pose_.data_.push_back(FramedTransformation(i, i, mat4d_I));
    }
    pose_visit.resize(frame_num_, false);
    return (traj_.data_.size() > 0);
}

void ProgressiveOpt::Opt() {
    std::map<key, std::vector<int>, key_comp> index_map;
    for (int i = 0; i < traj_.data_.size(); ++i)
    {
        int frame1 = traj_.data_[i].frame1_;
        int frame2 = traj_.data_[i].frame2_;
        index_map[key(frame1, frame2)].push_back(i);
    }
    std::vector<int>  seq(traj_.data_.size(),-1);

    if(loopDetect_.Init(traj_file_,frame_num_)){
        struct state{
            bool in_stack;
            int level;
            int start_vertex;
            state() :in_stack(false), level(-1), start_vertex(0){}
            state(bool ins, int le, int start) :in_stack(ins), level(le), start_vertex(start){}
        };
        std::vector<state> stack_vertex(frame_num_);

    }
}


