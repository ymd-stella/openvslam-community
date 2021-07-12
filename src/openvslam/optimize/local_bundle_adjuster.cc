#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/marker.h"
#include "openvslam/data/map_database.h"
#include "openvslam/optimize/local_bundle_adjuster.h"
#include "openvslam/optimize/internal/landmark_vertex_container.h"
#include "openvslam/optimize/internal/marker_vertex_container.h"
#include "openvslam/optimize/internal/se3/shot_vertex_container.h"
#include "openvslam/optimize/internal/se3/reproj_edge_wrapper.h"
#include "openvslam/optimize/internal/distance_edge.h"
#include "openvslam/util/converter.h"

#include <unordered_map>

#include <Eigen/StdVector>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

namespace openvslam {
namespace optimize {

local_bundle_adjuster::local_bundle_adjuster(const unsigned int num_first_iter,
                                             const unsigned int num_second_iter)
    : num_first_iter_(num_first_iter), num_second_iter_(num_second_iter) {}

void local_bundle_adjuster::optimize(openvslam::data::keyframe* curr_keyfrm, bool* const force_stop_flag) const {
    // 1. Aggregate the local and fixed keyframes, and local landmarks

    // Correct the local keyframes of the current keyframe
    std::unordered_map<unsigned int, data::keyframe*> local_keyfrms;

    local_keyfrms[curr_keyfrm->id_] = curr_keyfrm;
    const auto curr_covisibilities = curr_keyfrm->graph_node_->get_covisibilities();
    for (auto local_keyfrm : curr_covisibilities) {
        if (!local_keyfrm) {
            continue;
        }
        if (local_keyfrm->will_be_erased()) {
            continue;
        }

        local_keyfrms[local_keyfrm->id_] = local_keyfrm;
    }

    // Correct landmarks seen in local keyframes
    std::unordered_map<unsigned int, data::landmark*> local_lms;

    for (auto local_keyfrm : local_keyfrms) {
        const auto landmarks = local_keyfrm.second->get_landmarks();
        for (auto local_lm : landmarks) {
            if (!local_lm) {
                continue;
            }
            if (local_lm->will_be_erased()) {
                continue;
            }

            // Avoid duplication
            if (local_lms.count(local_lm->id_)) {
                continue;
            }

            local_lms[local_lm->id_] = local_lm;
        }
    }

    // Correct markers seen in local keyframes
    std::unordered_map<unsigned int, data::marker*> local_mkrs;

    for (auto local_keyfrm : local_keyfrms) {
        const auto markers = local_keyfrm.second->get_markers();
        for (auto local_mkr : markers) {
            if (!local_mkr) {
                continue;
            }

            // Avoid duplication
            if (local_mkrs.count(local_mkr->id_)) {
                continue;
            }

            local_mkrs[local_mkr->id_] = local_mkr;
        }
    }

    // Fixed keyframes: keyframes which observe local landmarks but which are NOT in local keyframes
    std::unordered_map<unsigned int, data::keyframe*> fixed_keyfrms;

    for (auto local_lm : local_lms) {
        const auto observations = local_lm.second->get_observations();
        for (auto& obs : observations) {
            auto fixed_keyfrm = obs.first;
            if (!fixed_keyfrm) {
                continue;
            }
            if (fixed_keyfrm->will_be_erased()) {
                continue;
            }

            // Do not add if it's in the local keyframes
            if (local_keyfrms.count(fixed_keyfrm->id_)) {
                continue;
            }

            // Avoid duplication
            if (fixed_keyfrms.count(fixed_keyfrm->id_)) {
                continue;
            }

            fixed_keyfrms[fixed_keyfrm->id_] = fixed_keyfrm;
        }
    }

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
    internal::se3::shot_vertex_container keyfrm_vtx_container(vtx_id_offset, local_keyfrms.size() + fixed_keyfrms.size());
    // Save the converted keyframes
    std::unordered_map<unsigned int, data::keyframe*> all_keyfrms;

    // Set the local keyframes to the optimizer
    for (auto& id_local_keyfrm_pair : local_keyfrms) {
        auto local_keyfrm = id_local_keyfrm_pair.second;

        all_keyfrms.emplace(id_local_keyfrm_pair);
        auto keyfrm_vtx = keyfrm_vtx_container.create_vertex(local_keyfrm, local_keyfrm->id_ == 0);
        optimizer.addVertex(keyfrm_vtx);
    }

    // Set the fixed keyframes to the optimizer
    for (auto& id_fixed_keyfrm_pair : fixed_keyfrms) {
        auto fixed_keyfrm = id_fixed_keyfrm_pair.second;

        all_keyfrms.emplace(id_fixed_keyfrm_pair);
        auto keyfrm_vtx = keyfrm_vtx_container.create_vertex(fixed_keyfrm, true);
        optimizer.addVertex(keyfrm_vtx);
    }

    // 4. Connect the vertices of the keyframe and the landmark by using an edge of reprojection constraint

    // Container of the landmark vertices
    internal::landmark_vertex_container lm_vtx_container(vtx_id_offset, local_lms.size());

    // Container of the reprojection edges
    using reproj_edge_wrapper = internal::se3::reproj_edge_wrapper<data::keyframe>;
    std::vector<reproj_edge_wrapper> reproj_edge_wraps;
    reproj_edge_wraps.reserve(all_keyfrms.size() * local_lms.size());

    // Chi-squared value with significance level of 5%
    // Two degree-of-freedom (n=2)
    constexpr float chi_sq_2D = 5.99146;
    const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);
    // Three degree-of-freedom (n=3)
    constexpr float chi_sq_3D = 7.81473;
    const float sqrt_chi_sq_3D = std::sqrt(chi_sq_3D);

    for (auto& id_local_lm_pair : local_lms) {
        auto local_lm = id_local_lm_pair.second;

        // Convert the landmark to the g2o vertex, then set to the optimizer
        auto lm_vtx = lm_vtx_container.create_vertex(local_lm, false);
        optimizer.addVertex(lm_vtx);

        const auto observations = local_lm->get_observations();
        for (const auto& obs : observations) {
            auto keyfrm = obs.first;
            auto idx = obs.second;
            if (!keyfrm) {
                continue;
            }
            if (keyfrm->will_be_erased()) {
                continue;
            }

            const auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
            const auto& undist_keypt = keyfrm->undist_keypts_.at(idx);
            const float x_right = keyfrm->stereo_x_right_.at(idx);
            const float inv_sigma_sq = keyfrm->inv_level_sigma_sq_.at(undist_keypt.octave);
            const auto sqrt_chi_sq = (keyfrm->camera_->setup_type_ == camera::setup_type_t::Monocular)
                                         ? sqrt_chi_sq_2D
                                         : sqrt_chi_sq_3D;
            auto reproj_edge_wrap = reproj_edge_wrapper(keyfrm, keyfrm_vtx, local_lm, lm_vtx,
                                                        idx, undist_keypt.pt.x, undist_keypt.pt.y, x_right,
                                                        inv_sigma_sq, sqrt_chi_sq);
            reproj_edge_wraps.push_back(reproj_edge_wrap);
            optimizer.addEdge(reproj_edge_wrap.edge_);
        }
    }

    // Container of the landmark vertices
    internal::marker_vertex_container marker_vtx_container(vtx_id_offset, local_mkrs.size());
    std::vector<reproj_edge_wrapper> mkr_reproj_edge_wraps;
    mkr_reproj_edge_wraps.reserve(all_keyfrms.size() * local_mkrs.size());

    for (auto& id_local_mkr_pair : local_mkrs) {
        auto mkr = id_local_mkr_pair.second;
        if (!mkr) {
            continue;
        }

        // Convert the corners to the g2o vertex, then set it to the optimizer
        auto corner_vertices = marker_vtx_container.create_vertices(mkr, false);
        // std::cout << "id: " << mkr->id_ << std::endl;
        for (unsigned int corner_idx = 0; corner_idx < corner_vertices.size(); ++corner_idx) {
            const auto corner_vtx = corner_vertices[corner_idx];
            // const Vec3_t pos_w = corner_vtx->estimate();
            // std::cout << "[" << pos_w[0] << ", " << pos_w[1] << ", " << pos_w[2] << "]" << std::endl;
            optimizer.addVertex(corner_vtx);

            for (const auto& keyfrm : mkr->observations_) {
                // TODO: キーフレームの削除をしていないので、チェックしないとキーフレームがculling済みの場合がある
                if (!keyfrm_vtx_container.contain(keyfrm)) {
                    continue;
                }
                const auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
                const auto& undist_pt = keyfrm->markers_2d_.find(mkr->id_)->second.undist_corners_.at(corner_idx);
                const float x_right = -1.0;
                const float inv_sigma_sq = 1.0;
                auto reproj_edge_wrap = reproj_edge_wrapper(keyfrm, keyfrm_vtx, nullptr, corner_vtx,
                                                            0, undist_pt.x, undist_pt.y, x_right,
                                                            inv_sigma_sq, 0.0, false);
                mkr_reproj_edge_wraps.push_back(reproj_edge_wrap);
                // reproj_edge_wrap.edge_->computeError();
                // std::cout << "chi2: " << reproj_edge_wrap.edge_->chi2() << std::endl;
                // std::cout << "error: " << Vec2_t{undist_pt.x, undist_pt.y} - ((internal::se3::equirectangular_reproj_edge*)reproj_edge_wrap.edge_)->cam_project(keyfrm_vtx->estimate().map(corner_vtx->estimate())) << std::endl;
                optimizer.addEdge(reproj_edge_wrap.edge_);
            }
        }
        // std::cout << std::endl;

        double informationRatio = 1.0e6;
        for (unsigned int corner_idx = 0; corner_idx < corner_vertices.size(); ++corner_idx) {
            unsigned int next_corner_idx = (corner_idx + 1) % corner_vertices.size();
            const auto dist_edge = new internal::distance_edge();
            const double marker_length = 0.24;
            dist_edge->setMeasurement(marker_length);
            dist_edge->setInformation(MatRC_t<1, 1>::Identity() * informationRatio);

            //debug
            // const Vec3_t pos_w1 = corner_vertices[corner_idx]->estimate();
            // const Vec3_t pos_w2 = corner_vertices[next_corner_idx]->estimate();
            // std::cout << "distance: " << (pos_w1 - pos_w2).norm() << std::endl;

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

    // 5. Perform the first optimization

    if (force_stop_flag) {
        if (*force_stop_flag) {
            return;
        }
    }

    optimizer.initializeOptimization();
    optimizer.optimize(num_first_iter_);

    // 6. Discard outliers, then perform the second optimization

    bool run_robust_BA = true;

    if (force_stop_flag) {
        if (*force_stop_flag) {
            run_robust_BA = false;
        }
    }

    if (run_robust_BA) {
        for (auto& reproj_edge_wrap : reproj_edge_wraps) {
            auto edge = reproj_edge_wrap.edge_;

            auto local_lm = reproj_edge_wrap.lm_;
            if (local_lm->will_be_erased()) {
                continue;
            }

            if (reproj_edge_wrap.is_monocular_) {
                if (chi_sq_2D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                    reproj_edge_wrap.set_as_outlier();
                }
            }
            else {
                if (chi_sq_3D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                    reproj_edge_wrap.set_as_outlier();
                }
            }

            edge->setRobustKernel(nullptr);
        }

        optimizer.initializeOptimization();
        optimizer.optimize(num_second_iter_);
    }

    // 7. Count the outliers

    std::vector<std::pair<data::keyframe*, data::landmark*>> outlier_observations;
    outlier_observations.reserve(reproj_edge_wraps.size());

    for (auto& reproj_edge_wrap : reproj_edge_wraps) {
        auto edge = reproj_edge_wrap.edge_;

        auto local_lm = reproj_edge_wrap.lm_;
        if (local_lm->will_be_erased()) {
            continue;
        }

        if (reproj_edge_wrap.is_monocular_) {
            if (chi_sq_2D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                outlier_observations.emplace_back(std::make_pair(reproj_edge_wrap.shot_, reproj_edge_wrap.lm_));
            }
        }
        else {
            if (chi_sq_3D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                outlier_observations.emplace_back(std::make_pair(reproj_edge_wrap.shot_, reproj_edge_wrap.lm_));
            }
        }
    }

    // 8. Update the information

    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        for (auto& outlier_obs : outlier_observations) {
            auto keyfrm = outlier_obs.first;
            auto lm = outlier_obs.second;
            keyfrm->erase_landmark(lm);
            lm->erase_observation(keyfrm);
        }

        for (auto id_local_keyfrm_pair : local_keyfrms) {
            auto local_keyfrm = id_local_keyfrm_pair.second;

            auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(local_keyfrm);
            local_keyfrm->set_cam_pose(keyfrm_vtx->estimate());
        }

        for (auto id_local_lm_pair : local_lms) {
            auto local_lm = id_local_lm_pair.second;

            auto lm_vtx = lm_vtx_container.get_vertex(local_lm);
            local_lm->set_pos_in_world(lm_vtx->estimate());
            local_lm->update_normal_and_depth();
        }

        for (auto id_local_mkr_pair : local_mkrs) {
            auto mkr = id_local_mkr_pair.second;
            if (!mkr) {
                continue;
            }

            // std::cout << "id: " << mkr->id_ << std::endl;
            for (unsigned int corner_idx = 0; corner_idx < 4; ++corner_idx) {
                auto corner_vtx = marker_vtx_container.get_vertex(mkr, corner_idx);
                const Vec3_t pos_w = corner_vtx->estimate();
                // std::cout << "[" << pos_w[0] << ", " << pos_w[1] << ", " << pos_w[2] << "]" << std::endl;
                mkr->set_corner_pos(pos_w, corner_idx);
            }

            // for (unsigned int corner_idx = 0; corner_idx < 4; ++corner_idx) {
            //     auto corner_vtx1 = marker_vtx_container.get_vertex(mkr, corner_idx);
            //     unsigned int next_corner_idx = (corner_idx + 1) % 4;
            //     auto corner_vtx2 = marker_vtx_container.get_vertex(mkr, next_corner_idx);
            //     const Vec3_t pos_w1 = corner_vtx1->estimate();
            //     const Vec3_t pos_w2 = corner_vtx2->estimate();
            //     std::cout << "distance: " << (pos_w1 - pos_w2).norm() << std::endl;
            // }
            // std::cout << std::endl;
        }
    }
}

} // namespace optimize
} // namespace openvslam
