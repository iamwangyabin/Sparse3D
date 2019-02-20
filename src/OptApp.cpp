//
// Created by wang on 19-1-30.
//

#include "OptApp.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include <g2o/solvers/dense/linear_solver_dense.h>



#include "g2o/core/jacobian_workspace.h"
#include "g2o/stuff/macros.h"

bool OptApp::Init() {
    namespace fs = boost::filesystem;
    if ( fs::exists( fs::path( loop_log_file_ ) ) ) {
        loop_traj_.LoadFromFile( loop_log_file_ );
        if ( fs::exists( fs::path( loop_info_file_ ) ) ) {
            loop_info_.LoadFromFile( loop_info_file_ );
        }
    }

    for (int i = 0; i < loop_traj_.data_.size(); ++i) {
        Eigen::Matrix4d temp = loop_traj_.data_[i].transformation_.inverse();
        loop_traj_.data_[i].transformation_=temp;
    }

    // all pose init to identity matrix
    if (fs::exists(fs::path(init_pose_file_))) {
        pose_traj_.LoadFromFile(init_pose_file_);
    }
    else
    {
        pose_traj_.data_.clear();
        for (int i = 0; i < frame_num_; ++i)
        {
            Eigen::Matrix4d mat4d_I = Eigen::Matrix4d::Identity();
            pose_traj_.data_.push_back(FramedTransformation(i, i, mat4d_I));
        }
    }

    return (loop_traj_.data_.size()>0);
}

void OptApp::OptimizeSwitchable() {

}

void OptApp::OptimizeSlam3d() {
    g2o::SparseOptimizer* optimizer;
    optimizer = new g2o::SparseOptimizer();
    optimizer->setVerbose(false);

    // create the linear solver
    auto linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
    // create the block solver on top of the linear solver
    auto solver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
    // create the algorithm to carry out the optimization
    g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(solver));
    optimizer->setAlgorithm(optimizationAlgorithm);
    Eigen::Matrix< double, 6, 6 > default_information;
    default_information = Eigen::Matrix< double, 6, 6 >::Identity();

    // 添加顶点
    for (int i = 0; i < frame_num_; i++) {
        g2o::VertexSE3 * v = new g2o::VertexSE3();
        v->setId(i);
        // 预设值
        v->setEstimate(Eigen2G2O(pose_traj_.data_[i].transformation_));
        // 第一个点固定为零
        if (i == 0) {
            v->setFixed(true);
        }
        optimizer->addVertex(v);
    }
    // 添加边
    std::vector<double> init_error_arr;
    for (int i = 0; i < (int)loop_traj_.data_.size() ; ++i) {
        FramedTransformation & t = loop_traj_.data_[i];

        g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3();
        // config edge vertice
        g2o_edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex(t.frame1_));
        g2o_edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex(t.frame2_));
        g2o_edge->setMeasurement(g2o::internal::fromSE3Quat(Eigen2G2O(t.transformation_)));

        auto f = g2o::
    }

}

