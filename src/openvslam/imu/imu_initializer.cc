#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/imu/imu_initializer.h"
#include "openvslam/imu/preintegrator.h"
#include "openvslam/imu/internal/velocity_vertex_container.h"
#include "openvslam/imu/internal/bias_vertex_container.h"
#include "openvslam/imu/internal/prior_bias_edge_wrapper.h"
#include "openvslam/imu/internal/gravity_dir_vertex.h"
#include "openvslam/imu/internal/scale_vertex.h"
#include "openvslam/imu/internal/inertial_gravity_scale_edge_on_imu_wrapper.h"
#include "openvslam/optimize/internal/se3/shot_vertex_container.h"

#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include <spdlog/spdlog.h>

namespace openvslam {
namespace optimize {

imu_initializer::imu_initializer(const unsigned int num_iter)
    : num_iter_(num_iter) {}

bool imu_initializer::initialize(const std::vector<data::keyframe*>& keyfrms, Mat33_t& Rwg, double& scale,
                                 bool depth_is_avaliable, float info_prior_gyr, float info_prior_acc) const {
    // 1. Construct an optimizer

    auto linear_solver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
    auto block_solver = g2o::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    algorithm->setUserLambdaInit(1e3);

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algorithm);

    // 2. Convert each of the keyframe to the g2o vertex, then set it to the optimizer

    // Container of the shot vertices
    auto vtx_id_offset = std::make_shared<unsigned int>(0);
    internal::se3::shot_vertex_container imu_pose_vtx_container(vtx_id_offset, keyfrms.size());
    imu::internal::velocity_vertex_container velocity_vtx_container(vtx_id_offset, keyfrms.size());
    imu::internal::bias_vertex_container gyr_bias_vtx_container(vtx_id_offset, 1);
    imu::internal::bias_vertex_container acc_bias_vtx_container(vtx_id_offset, 1);

    // Set the keyframes to the optimizer
    for (const auto keyfrm : keyfrms) {
        if (!keyfrm) {
            continue;
        }
        if (keyfrm->will_be_erased()) {
            continue;
        }

        auto imu_pose_vtx = imu_pose_vtx_container.create_vertex(keyfrm->id_, keyfrm->get_imu_pose(), true);
        optimizer.addVertex(imu_pose_vtx);

        auto velocity_vtx = velocity_vtx_container.create_vertex(keyfrm, false);
        optimizer.addVertex(velocity_vtx);
    }

    auto gyr_bias_vtx = gyr_bias_vtx_container.create_vertex(keyfrms.back()->id_, keyfrms.back()->imu_bias_.gyr_, false);
    optimizer.addVertex(gyr_bias_vtx);
    auto acc_bias_vtx = acc_bias_vtx_container.create_vertex(keyfrms.back()->id_, keyfrms.back()->imu_bias_.acc_, false);
    optimizer.addVertex(acc_bias_vtx);

    // Gravity and scale
    auto gravity_dir_vtx = new imu::internal::gravity_dir_vertex();
    gravity_dir_vtx->setId(*vtx_id_offset);
    (*vtx_id_offset)++;
    gravity_dir_vtx->setEstimate(Rwg);
    optimizer.addVertex(gravity_dir_vtx);

    auto scale_vtx = new imu::internal::scale_vertex();
    scale_vtx->setId(*vtx_id_offset);
    (*vtx_id_offset)++;
    scale_vtx->setEstimate(scale);
    scale_vtx->setFixed(depth_is_avaliable);
    optimizer.addVertex(scale_vtx);

    // 3. Connect the vertices of the keyframe and the imu data

    // Add prior to comon biases
    imu::internal::prior_bias_edge_wrapper pba_edge_wrap(info_prior_acc, acc_bias_vtx);
    optimizer.addEdge(pba_edge_wrap.edge_);
    imu::internal::prior_bias_edge_wrapper pbg_edge_wrap(info_prior_gyr, gyr_bias_vtx);
    optimizer.addEdge(pbg_edge_wrap.edge_);

    // IMU links with gravity and scale
    std::vector<imu::internal::inertial_gravity_scale_edge_on_imu_wrapper> inertial_gravity_scale_edge_wraps;
    inertial_gravity_scale_edge_wraps.reserve(keyfrms.size());

    for (size_t i = 0; i < keyfrms.size(); i++) {
        auto keyfrm = keyfrms.at(i);
        if (!keyfrm) {
            continue;
        }
        if (keyfrm->will_be_erased()) {
            continue;
        }
        if (!keyfrm->inertial_ref_keyfrm_) {
            continue;
        }
        assert(keyfrm->imu_preintegrator_from_inertial_ref_keyfrm_);
        auto ref_keyfrm = keyfrm->inertial_ref_keyfrm_;

        auto imu_pose_vtx1 = imu_pose_vtx_container.get_vertex(ref_keyfrm);
        auto velocity_vtx1 = velocity_vtx_container.get_vertex(ref_keyfrm);
        auto imu_pose_vtx2 = imu_pose_vtx_container.get_vertex(keyfrm);
        auto velocity_vtx2 = velocity_vtx_container.get_vertex(keyfrm);
        imu::internal::inertial_gravity_scale_edge_on_imu_wrapper inertial_gravity_scale_edge_wrap(
            keyfrm->imu_preintegrator_from_inertial_ref_keyfrm_->preintegrated_,
            acc_bias_vtx, gyr_bias_vtx, imu_pose_vtx1, velocity_vtx1,
            imu_pose_vtx2, velocity_vtx2, gravity_dir_vtx, scale_vtx);

        inertial_gravity_scale_edge_wraps.push_back(inertial_gravity_scale_edge_wrap);
        optimizer.addEdge(inertial_gravity_scale_edge_wrap.edge_);
    }

    // 4. Perform optimization

    optimizer.initializeOptimization();
    optimizer.optimize(num_iter_);

    // 5. Extract the result
    scale = scale_vtx->estimate();

    if (scale < 1e-1) {
        spdlog::warn("scale too small {}", scale);
        return false;
    }
    if (scale > 1e+1) {
        spdlog::warn("scale too large {}", scale);
        return false;
    }

    Rwg = gravity_dir_vtx->estimate();

    Vec3_t bias_acc = acc_bias_vtx->estimate();
    Vec3_t bias_gyr = gyr_bias_vtx->estimate();
    imu::bias b(bias_acc, bias_gyr);
    for (auto keyfrm : keyfrms) {
        if (velocity_vtx_container.contain(keyfrm)) {
            auto velocity_vtx = static_cast<imu::internal::velocity_vertex*>(velocity_vtx_container.get_vertex(keyfrm));
            keyfrm->velocity_ = velocity_vtx->estimate();
        }

        double tolerance = 0.01;
        if ((keyfrm->imu_bias_.gyr_ - b.gyr_).norm() > tolerance) {
            keyfrm->imu_bias_ = b;
            if (keyfrm->imu_preintegrator_from_inertial_ref_keyfrm_) {
                keyfrm->imu_preintegrator_from_inertial_ref_keyfrm_->reintegrate(b);
            }
        }
        else {
            keyfrm->imu_bias_ = b;
        }
    }
    return true;
}

} // namespace optimize
} // namespace openvslam
