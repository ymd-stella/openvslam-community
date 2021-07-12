#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/marker.h"
#include "openvslam/data/map_database.h"
#include "openvslam/optimize/global_bundle_adjuster.h"
#include "openvslam/optimize/internal/landmark_vertex_container.h"
#include "openvslam/optimize/internal/marker_vertex_container.h"
#include "openvslam/optimize/internal/se3/shot_vertex_container.h"
#include "openvslam/optimize/internal/se3/reproj_edge_wrapper.h"
#include "openvslam/optimize/internal/distance_edge.h"
#include "openvslam/util/converter.h"

#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

namespace openvslam {
namespace optimize {

global_bundle_adjuster::global_bundle_adjuster(data::map_database* map_db, const unsigned int num_iter, const bool use_huber_kernel)
    : map_db_(map_db), num_iter_(num_iter), use_huber_kernel_(use_huber_kernel) {}

void global_bundle_adjuster::optimize(const unsigned int lead_keyfrm_id_in_global_BA, bool* const force_stop_flag) const {
    // 1. Collect the dataset

    const auto keyfrms = map_db_->get_all_keyframes();
    const auto lms = map_db_->get_all_landmarks();
    const auto markers = map_db_->get_all_markers();
    std::vector<bool> is_optimized_lm(lms.size(), true);

    // 2. Construct an optimizer

    auto linear_solver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
    auto block_solver = g2o::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algorithm);

    if (force_stop_flag) {
        optimizer.setForceStopFlag(force_stop_flag);
    }

    // 3. Convert each of the keyframe to the g2o vertex, then set it to the optimizer

    // Container of the shot vertices
    auto vtx_id_offset = std::make_shared<unsigned int>(0);
    internal::se3::shot_vertex_container keyfrm_vtx_container(vtx_id_offset, keyfrms.size());

    // Set the keyframes to the optimizer
    for (const auto keyfrm : keyfrms) {
        if (!keyfrm) {
            continue;
        }
        if (keyfrm->will_be_erased()) {
            continue;
        }

        auto keyfrm_vtx = keyfrm_vtx_container.create_vertex(keyfrm, keyfrm->id_ == 0);
        optimizer.addVertex(keyfrm_vtx);
    }

    // 4. Connect the vertices of the keyframe and the landmark by using reprojection edge

    // Container of the landmark vertices
    internal::landmark_vertex_container lm_vtx_container(vtx_id_offset, lms.size());

    // Container of the reprojection edges
    using reproj_edge_wrapper = internal::se3::reproj_edge_wrapper<data::keyframe>;
    std::vector<reproj_edge_wrapper> reproj_edge_wraps;
    reproj_edge_wraps.reserve(10 * lms.size());

    // Chi-squared value with significance level of 5%
    // Two degree-of-freedom (n=2)
    constexpr float chi_sq_2D = 5.99146;
    const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);
    // Three degree-of-freedom (n=3)
    constexpr float chi_sq_3D = 7.81473;
    const float sqrt_chi_sq_3D = std::sqrt(chi_sq_3D);

    for (unsigned int i = 0; i < lms.size(); ++i) {
        auto lm = lms.at(i);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        // Convert the landmark to the g2o vertex, then set it to the optimizer
        auto lm_vtx = lm_vtx_container.create_vertex(lm, false);
        optimizer.addVertex(lm_vtx);

        unsigned int num_edges = 0;
        const auto observations = lm->get_observations();
        for (const auto& obs : observations) {
            auto keyfrm = obs.first;
            auto idx = obs.second;
            if (!keyfrm) {
                continue;
            }
            if (keyfrm->will_be_erased()) {
                continue;
            }

            if (!keyfrm_vtx_container.contain(keyfrm)) {
                continue;
            }

            const auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
            const auto& undist_keypt = keyfrm->undist_keypts_.at(idx);
            const float x_right = keyfrm->stereo_x_right_.at(idx);
            const float inv_sigma_sq = keyfrm->inv_level_sigma_sq_.at(undist_keypt.octave);
            const auto sqrt_chi_sq = (keyfrm->camera_->setup_type_ == camera::setup_type_t::Monocular)
                                         ? sqrt_chi_sq_2D
                                         : sqrt_chi_sq_3D;
            auto reproj_edge_wrap = reproj_edge_wrapper(keyfrm, keyfrm_vtx, lm, lm_vtx,
                                                        idx, undist_keypt.pt.x, undist_keypt.pt.y, x_right,
                                                        inv_sigma_sq, sqrt_chi_sq, use_huber_kernel_);
            reproj_edge_wraps.push_back(reproj_edge_wrap);
            optimizer.addEdge(reproj_edge_wrap.edge_);
            ++num_edges;
        }

        if (num_edges == 0) {
            optimizer.removeVertex(lm_vtx);
            is_optimized_lm.at(i) = false;
        }
    }

    // Container of the landmark vertices
    internal::marker_vertex_container marker_vtx_container(vtx_id_offset, markers.size());

    // Container of the reprojection edges
    // using reproj_edge_wrapper = internal::se3::reproj_edge_wrapper<data::keyframe>;
    // std::vector<reproj_edge_wrapper> reproj_edge_wraps;
    // reproj_edge_wraps.reserve(10 * lms.size());

    for (unsigned int marker_idx = 0; marker_idx < markers.size(); ++marker_idx) {
        auto mkr = markers.at(marker_idx);
        if (!mkr) {
            continue;
        }

        // Convert the corners to the g2o vertex, then set it to the optimizer
        auto corner_vertices = marker_vtx_container.create_vertices(mkr, false);
        std::cout << "id: " << mkr->id_ << std::endl;
        for (unsigned int corner_idx = 0; corner_idx < corner_vertices.size(); ++corner_idx) {
            const auto corner_vtx = corner_vertices[corner_idx];
            const Vec3_t pos_w = corner_vtx->estimate();
            std::cout << "[" << pos_w[0] << ", " << pos_w[1] << ", " << pos_w[2] << "]" << std::endl;
            optimizer.addVertex(corner_vtx);

            for (const auto& keyfrm : mkr->observations_) {
                const auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
                const auto& undist_pt = keyfrm->markers_2d_.find(mkr->id_)->second.undist_corners_.at(corner_idx);
                const float x_right = -1.0;
                const float inv_sigma_sq = 1.0;
                auto reproj_edge_wrap = reproj_edge_wrapper(keyfrm, keyfrm_vtx, nullptr, corner_vtx,
                                                            0, undist_pt.x, undist_pt.y, x_right,
                                                            inv_sigma_sq, 0.0, false);
                reproj_edge_wraps.push_back(reproj_edge_wrap);
                reproj_edge_wrap.edge_->computeError();
                std::cout << "chi2: " << reproj_edge_wrap.edge_->chi2() << std::endl;
                std::cout << "error: " << Vec2_t{undist_pt.x, undist_pt.y} - ((internal::se3::equirectangular_reproj_edge*)reproj_edge_wrap.edge_)->cam_project(keyfrm_vtx->estimate().map(corner_vtx->estimate())) << std::endl;
                optimizer.addEdge(reproj_edge_wrap.edge_);
            }
        }
        std::cout << std::endl;

        double informationRatio = 1.0e6;
        for (unsigned int corner_idx = 0; corner_idx < corner_vertices.size(); ++corner_idx) {
            unsigned int next_corner_idx = (corner_idx + 1) % corner_vertices.size();
            const auto dist_edge = new internal::distance_edge();
            const double marker_length = 0.24;
            dist_edge->setMeasurement(marker_length);
            dist_edge->setInformation(MatRC_t<1, 1>::Identity() * informationRatio);

            //debug
            const Vec3_t pos_w1 = corner_vertices[corner_idx]->estimate();
            const Vec3_t pos_w2 = corner_vertices[next_corner_idx]->estimate();
            std::cout << "distance: " << (pos_w1 - pos_w2).norm() << std::endl;

            dist_edge->setVertex(0, corner_vertices[corner_idx]);
            dist_edge->setVertex(1, corner_vertices[next_corner_idx]);
            optimizer.addEdge(dist_edge);
        }
        for (unsigned int corner_idx = 0; corner_idx < 2; ++corner_idx) {
            unsigned int next_corner_idx = (corner_idx + 2) % corner_vertices.size();
            const auto dist_edge = new internal::distance_edge();
            const double marker_length = 0.24;
            const double diagonal_length = std::sqrt(2) * marker_length;
            dist_edge->setMeasurement(diagonal_length);
            dist_edge->setInformation(MatRC_t<1, 1>::Identity() * informationRatio);
            dist_edge->setVertex(0, corner_vertices[corner_idx]);
            dist_edge->setVertex(1, corner_vertices[next_corner_idx]);
            optimizer.addEdge(dist_edge);
        }
    }

    // 5. Perform optimization

    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(num_iter_);

    std::cout << "edges: " << reproj_edge_wraps.size() << std::endl;
    for (const auto& reproj_edge_wrap : reproj_edge_wraps) {
        reproj_edge_wrap.edge_->computeError();
        std::cout << "chi2: " << reproj_edge_wrap.edge_->chi2() << std::endl;
    }

    if (force_stop_flag && *force_stop_flag) {
        return;
    }

    // 6. Extract the result

    for (auto keyfrm : keyfrms) {
        if (keyfrm->will_be_erased()) {
            continue;
        }
        auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
        const auto cam_pose_cw = util::converter::to_eigen_mat(keyfrm_vtx->estimate());
        if (lead_keyfrm_id_in_global_BA == 0) {
            keyfrm->set_cam_pose(cam_pose_cw);
        }
        else {
            keyfrm->cam_pose_cw_after_loop_BA_ = cam_pose_cw;
            keyfrm->loop_BA_identifier_ = lead_keyfrm_id_in_global_BA;
        }
    }

    for (unsigned int i = 0; i < lms.size(); ++i) {
        if (!is_optimized_lm.at(i)) {
            continue;
        }

        auto lm = lms.at(i);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        auto lm_vtx = lm_vtx_container.get_vertex(lm);
        const Vec3_t pos_w = lm_vtx->estimate();

        if (lead_keyfrm_id_in_global_BA == 0) {
            lm->set_pos_in_world(pos_w);
            lm->update_normal_and_depth();
        }
        else {
            lm->pos_w_after_global_BA_ = pos_w;
            lm->loop_BA_identifier_ = lead_keyfrm_id_in_global_BA;
        }
    }

    for (unsigned int marker_idx = 0; marker_idx < markers.size(); ++marker_idx) {
        auto mkr = markers.at(marker_idx);
        if (!mkr) {
            continue;
        }

        std::cout << "id: " << mkr->id_ << std::endl;
        for (unsigned int corner_idx = 0; corner_idx < 4; ++corner_idx) {
            auto corner_vtx = marker_vtx_container.get_vertex(mkr, corner_idx);
            const Vec3_t pos_w = corner_vtx->estimate();
            std::cout << "[" << pos_w[0] << ", " << pos_w[1] << ", " << pos_w[2] << "]" << std::endl;
            mkr->set_corner_pos(pos_w, corner_idx);
        }

        for (unsigned int corner_idx = 0; corner_idx < 4; ++corner_idx) {
            auto corner_vtx1 = marker_vtx_container.get_vertex(mkr, corner_idx);
            unsigned int next_corner_idx = (corner_idx + 1) % 4;
            auto corner_vtx2 = marker_vtx_container.get_vertex(mkr, next_corner_idx);
            const Vec3_t pos_w1 = corner_vtx1->estimate();
            const Vec3_t pos_w2 = corner_vtx2->estimate();
            std::cout << "distance: " << (pos_w1 - pos_w2).norm() << std::endl;
        }
        std::cout << std::endl;
    }
}

} // namespace optimize
} // namespace openvslam
