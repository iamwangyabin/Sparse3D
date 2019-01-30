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
using namespace g2o;
void OptApp::OptimizeSlam3d() {
//    g2o::SparseOptimizer* optimizer;
//    optimizer = new g2o::SparseOptimizer();
//    optimizer->setVerbose(true);
//    std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver(g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>());
//    g2o::BlockSolverX::LinearSolverType * linearSolver;
//
//    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
//    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
//    g2o::BlockSolverX* solver =new g2o::BlockSolverX(linearSolver);
//    g2o::BlockSolverX* solver;
//    solver_ptr=new BalBlockSolver(std::unique_ptr<BalBlockSolver::LinearSolverType>(linearSolver));
//    solver = new g2o::BlockSolverX(std::unique_ptr<g2o::BlockSolverX::PoseMatrixType> (linearSolver));
//    std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverCSparse<Block::PoseMatrixType>());
//
//    //Block* solver_ptr = new Block ( linearSolver );
//    //std::unique_ptr<Block> solver_ptr ( new Block ( linearSolver));
//    std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));
//    g2o::LinearSolverType * linearSolver = g2o::LinearSolverCSparse<g2o::BlockSolverXPoseMatrixType> ();

//    BlockSolverX::LinearSolverType * linearSolver = new LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
//    BlockSolverX * solver_ptr = new BlockSolverX(linearSolver);
    auto linearSolver = g2o::make_unique<LinearSolverCSparse<BlockSolverX::PoseMatrixType>>();

    // create the block solver on top of the linear solver
    auto blockSolver = g2o::make_unique<BlockSolverX>(std::move(linearSolver));
}

