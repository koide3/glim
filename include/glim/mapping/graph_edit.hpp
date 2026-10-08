#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam_points/optimizers/isam2_result_ext.hpp>

#include <glim/mapping/graph_metadata.hpp>

namespace glim {

/**
 * @brief State of the graph editing workflow
 */
enum class GraphEditState {
  IDLE,
  SESSION_MERGE_PENDING,
  CANDIDATE_EDITING,
};

/**
 * @brief Keys owned by one submap
 *
 * A submap owns one origin pose and two endpoint pose, velocity, and bias states.
 */
struct SubmapStateKeys {
  gtsam::Key origin_pose;                    ///< Origin pose X(i)
  std::array<gtsam::Key, 2> endpoint_poses;  ///< Endpoint poses E(2i), E(2i+1)
  std::array<gtsam::Key, 2> velocities;      ///< Endpoint velocities V(2i), V(2i+1)
  std::array<gtsam::Key, 2> biases;          ///< Endpoint biases B(2i), B(2i+1)

  /**
   * @brief Get all keys in the state block
   * @return Keys in X/E/V/B order
   */
  gtsam::KeyVector all() const;
};

/**
 * @brief Get the complete state keys owned by a submap
 * @param submap_id Submap ID
 * @return Complete X/E/V/B key block
 */
SubmapStateKeys submap_state_keys(int submap_id);

/**
 * @brief Collect complete state keys for multiple submaps
 * @param submap_ids Submap IDs
 * @return Deduplicated X/E/V/B keys
 */
gtsam::KeySet collect_submap_state_keys(const std::vector<int>& submap_ids);

/**
 * @brief Copy values except those in the removal set
 * @param values Input values
 * @param removed_keys Keys to remove
 * @return Filtered values
 */
gtsam::Values filter_values_by_keys(const gtsam::Values& values, const gtsam::KeySet& removed_keys);

/**
 * @brief Copy factors that do not touch any removed key
 * @param factors Input factors
 * @param removed_keys Keys to remove
 * @return Filtered factors
 */
gtsam::NonlinearFactorGraph filter_factors_by_keys(const gtsam::NonlinearFactorGraph& factors, const gtsam::KeySet& removed_keys);

/**
 * @brief Validate that every factor is non-null and references existing values
 * @param factors Factors to validate
 * @param values Values available to the factors
 * @throws std::invalid_argument if a factor is null or references a missing key
 */
void validate_factor_keys(const gtsam::NonlinearFactorGraph& factors, const gtsam::Values& values);

/**
 * @brief Full pose gauge anchor information
 */
struct PoseGaugeAnchor {
  enum class Type {
    POSE_PRIOR,
    LINEAR_DAMPING,
  };

  Type type;
  gtsam::Key key;                       ///< Anchored X key
  gtsam::Pose3 prior;                   ///< Prior mean for a pose prior
  gtsam::SharedNoiseModel noise_model;  ///< Pose prior noise model
  gtsam::Vector damping_diagonal;       ///< Hessian diagonal for linear damping
};

/**
 * @brief Find full pose gauge anchors
 * @param factors Factors to inspect
 * @return Full pose gauge anchors on X keys
 */
std::vector<PoseGaugeAnchor> find_pose_gauge_anchors(const gtsam::NonlinearFactorGraph& factors);

/**
 * @brief Select the primary gauge pose from full pose anchors
 * @param anchors Full pose anchors to inspect
 * @return Anchored X key with the lowest submap ID
 * @throws std::invalid_argument if no anchor is available
 */
gtsam::Key select_pose_gauge_anchor_key(const std::vector<PoseGaugeAnchor>& anchors);

/**
 * @brief Preserve or deterministically transfer the full pose gauge anchor
 * @param candidate_factors Candidate factors to update
 * @param candidate_values Candidate values after pruning
 * @param original_anchors Anchors from the target graph
 * @param active_target_submaps Active target submap IDs
 *
 * If the primary gauge pose was pruned, all of its anchors are transferred to
 * the current pose of the lowest active target submap ID.
 */
void ensure_pose_gauge_anchor(
  gtsam::NonlinearFactorGraph& candidate_factors,
  const gtsam::Values& candidate_values,
  const std::vector<PoseGaugeAnchor>& original_anchors,
  const std::vector<int>& active_target_submaps);

/**
 * @brief Connectivity information for a complete key-factor graph
 */
struct GraphConnectivity {
  std::vector<gtsam::KeyVector> components;  ///< Components containing all value keys
  gtsam::KeyVector reachable_keys;           ///< Keys reachable from the gauge anchor
  gtsam::KeyVector unreachable_pose_keys;    ///< X keys not reachable from the anchor
  std::size_t pose_component_count = 0;      ///< Number of components containing X keys

  /// @brief Check whether every pose is reachable from the gauge anchor
  bool all_poses_reachable() const { return unreachable_pose_keys.empty(); }

  /// @brief Count pose components not containing the gauge anchor
  std::size_t orphaned_subgraph_count() const { return all_poses_reachable() ? 0 : pose_component_count - 1; }
};

/**
 * @brief Analyze graph connectivity using factors as hyperedges over value keys
 * @param values Graph values
 * @param factors Graph factors
 * @param anchor_key Full pose gauge anchor key
 * @return Complete key components and pose reachability diagnostics
 */
GraphConnectivity analyze_graph_connectivity(const gtsam::Values& values, const gtsam::NonlinearFactorGraph& factors, gtsam::Key anchor_key);

/**
 * @brief Result of building and validating a graph in a temporary iSAM2
 */
struct TrialISAM2Build {
  std::unique_ptr<gtsam_points::ISAM2Ext> optimizer;  ///< Temporary optimizer
  gtsam_points::ISAM2ResultExt update_result;         ///< Initial update result
  gtsam::Values estimate;                             ///< Complete optimized estimate
};

/**
 * @brief Options captured when committing a pending session merge
 */
struct SessionMergeOptions {
  gtsam::NonlinearFactor::shared_ptr merge_factor;  ///< User-confirmed target-to-source merge factor
  std::vector<SubmapRange> prune_ranges;            ///< Target and source submap ranges to prune
};

/**
 * @brief Candidate graph produced by a graph edit transaction
 */
struct CandidateGraph {
  gtsam::NonlinearFactorGraph factors;
  gtsam::Values values;

  std::vector<SubmapRange> requested_prune_ranges;
  std::vector<SubmapRange> applied_prune_ranges;

  GraphConnectivity connectivity;

  std::string diagnostic;
};

/**
 * @brief Append factors to a graph edit candidate and refresh its connectivity
 * @param candidate Candidate graph to update
 * @param factors Factors whose keys must already exist in the candidate
 */
void append_candidate_factors(CandidateGraph& candidate, const gtsam::NonlinearFactorGraph& factors);

/**
 * @brief Apply a global rigid transform to a session's complete state
 * @param values Session values
 * @param transform Global transform applied to X/E and whose rotation is applied to V
 * @return Transformed values with biases and unknown value types unchanged
 */
gtsam::Values transform_session_values(const gtsam::Values& values, const gtsam::Pose3& transform);

/**
 * @brief Build a filtered session merge candidate without modifying the target graph
 * @param target_factors Current target graph factors
 * @param target_values Current target graph values
 * @param source_factors Additional source session factors
 * @param source_values Additional source session values
 * @param source_begin First submap ID owned by the source session
 * @param total_submaps Total target and source submap count
 * @param options Frozen session merge options
 * @return Candidate graph and connectivity diagnostics
 */
CandidateGraph build_session_merge_candidate(
  const gtsam::NonlinearFactorGraph& target_factors,
  const gtsam::Values& target_values,
  const gtsam::NonlinearFactorGraph& source_factors,
  const gtsam::Values& source_values,
  int source_begin,
  int total_submaps,
  const SessionMergeOptions& options);

/**
 * @brief Build and validate a graph in a fresh iSAM2 instance
 * @param factors Complete candidate factors
 * @param values Complete candidate initial values
 * @param params Parameters shared with the active optimizer
 * @return Temporary optimizer, update result, and complete estimate
 *
 * Factor/value consistency is validated before insertion. Exceptions are
 * intentionally propagated to the transaction caller. The active optimizer
 * is never accessed or modified by this function.
 */
TrialISAM2Build build_trial_isam2(const gtsam::NonlinearFactorGraph& factors, const gtsam::Values& values, const gtsam::ISAM2Params& params);

}  // namespace glim
