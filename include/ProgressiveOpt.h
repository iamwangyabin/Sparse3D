//
// Created by wang on 19-1-24.
//

#ifndef SPARSE3D_PROGRESSIVEOPT_H
#define SPARSE3D_PROGRESSIVEOPT_H

#include <queue>
#include <map>
#include <vector>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/isometry3d_gradients.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <boost/filesystem.hpp>

#include "common_include.h"
#include "GraphLoopDetect.h"

/**
 * 就是用来构造indexmap的
 * 需要提供自定义的比较类型
 * */
struct key{
    int frame1, frame2;
    key() :frame1(0), frame2(0){}
    key(int id1, int id2) :frame1(id1), frame2(id2){}
};

struct key_comp{
    bool operator () (const key &k1, const key &k2)
    {
        return k1.frame1 < k2.frame1 || (k1.frame1 == k2.frame1 && k1.frame2 < k2.frame2);
    }
};

struct inx2seq{
    int index;
    int seq;
    inx2seq(int inx, int s) :index(inx), seq(s){}
};

struct inx2seq_cmp{
    bool operator()(inx2seq& i1, inx2seq& i2)
    {
        return i1.index < i2.index;
    }
};


class ProgressiveOpt{
    int end_flag;
    int Bnum;

    double score_threshold1_;
    double score_threshold2_;
    double error_threshold_;
    double low_score_threshold_;

    SeqSaveAndLoad helpSeq_;
    std::vector<bool> pose_visit;
    std::vector<std::vector<key>> select_loop;
public:
    GraphLoopDetect loopDetect_;
    RGBDTrajectory traj_;
    RGBDInformation2 info_;
    RGBDTrajectory pose_;

    // input
    std::string traj_file_;
    std::string info_file_;
    int frame_num_;

    // output
    std::string pose_file_;
    std::string fail_file_;
    std::string selected_edge_file_;

    ProgressiveOpt(int frame_num, std::string traj_file, std::string info_file, std::string pose_file, std::string fail_file, std::string selected_edge_file, double low_score_threshold, double score1, double score2, double error_threshold);
    ~ProgressiveOpt(){}

    bool Init();
    void Opt();

private:
    std::vector<int> BeamSearch(
            const double err_threshold,
            const int beam_width,
            std::map<key, std::vector<int>, key_comp>& index_map,
            const std::vector<key>& loop, const std::vector<int>& seq
            );

    void LoopError(
            const std::vector<std::vector<inx2seq>>& all_loop,
            int loop_len,
            /*output parameter*/
            double& min_error,
            std::vector<inx2seq>& best_loop);

    void ComputePose(const std::vector<int>& seq);
    void ComputeInitPose(const std::vector<int>& seq);

    /*--------- use some strategies to link all frames after loop search. ---------*/
    // use sequential and score to link.
    void UnionFindSet(std::vector<int>& seq);
    // only use score to link.
    void UnionFindSetNonSequential(std::vector<int>& seq);

    bool judgeLinkAllVertex(std::vector<int>& seq, RGBDTrajectory& loop);

    Eigen::Matrix4d G2O2Matrix4d(const g2o::SE3Quat& se3) {
        Eigen::Matrix4d m = se3.to_homogeneous_matrix(); //_Matrix< 4, 4, double >
        return m;
    }



    g2o::SE3Quat Eigen2G2O(const Eigen::Matrix4d & eigen_mat) {
        Eigen::Affine3d eigen_transform(eigen_mat);
        Eigen::Quaterniond eigen_quat(eigen_transform.rotation());
        Eigen::Vector3d translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
        g2o::SE3Quat result(eigen_quat, translation);
        return result;
    }

    void recur_seq(std::vector<int> frag_len, std::vector<int> &inx, int level, std::vector<inx2seq> item,
                   std::vector<std::vector<inx2seq>> &result, const int width);
};


#endif //SPARSE3D_PROGRESSIVEOPT_H
