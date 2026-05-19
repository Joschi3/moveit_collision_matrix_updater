#ifndef MOVEIT_COLLISION_MATRIX_UPDATER_MOVEIT_COLLISION_MATRIX_UPDATER_HPP
#define MOVEIT_COLLISION_MATRIX_UPDATER_MOVEIT_COLLISION_MATRIX_UPDATER_HPP

#pragma once

/*
 * Collision Matrix Updater (core API)
 *
 * This header declares the functions that:
 *  - initialize the MoveIt setup DataWarehouse
 *  - initialize the DefaultCollisions updater
 *  - recompute the collision matrix
 *  - write the SRDF to disk
 */

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// MoveIt Setup Framework
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>

// Collision Computation
#include <moveit_setup_srdf_plugins/default_collisions.hpp>

namespace collision_matrix_updater
{

struct NamedPoseCollision {
  std::string link1;
  std::string link2;
  std::string pose_name;
};

/**
 * @brief Initialize the DataWarehouse with URDF and SRDF configurations
 * @param parent_node The ROS2 node
 * @param urdf_path Path to the URDF file
 * @param srdf_path Path to the SRDF file
 * @return Configured DataWarehouse
 */
moveit_setup::DataWarehousePtr initializeDataWarehouse( const rclcpp::Node::SharedPtr &parent_node,
                                                        const std::string &urdf_path,
                                                        const std::string &srdf_path );

/**
 * @brief Initialize the DefaultCollisions updater
 * @param data_warehouse The configured data warehouse
 * @return DefaultCollisions instance (caller owns and must delete)
 */
moveit_setup::srdf_setup::DefaultCollisions *
initializeCollisionUpdater( const moveit_setup::DataWarehousePtr &data_warehouse );

/**
 * @brief Recalculate and update the collision matrix in the SRDF
 * @param collision_updater The DefaultCollisions instance
 * @param srdf_config The SRDF configuration
 * @param num_trials Number of collision checking trials
 * @param min_collision_fraction Minimum collision fraction threshold
 */
void updateCollisionMatrix( moveit_setup::srdf_setup::DefaultCollisions *collision_updater,
                            const std::shared_ptr<moveit_setup::SRDFConfig> &srdf_config,
                            unsigned int num_trials = 1000, double min_collision_fraction = 0.05 );

/**
 * @brief Write the updated SRDF file
 * @param srdf_config The SRDF configuration
 * @param output_path Path where the SRDF file will be written
 */
void writeSRDFFile( const std::shared_ptr<moveit_setup::SRDFConfig> &srdf_config,
                    const std::string &output_path );

/**
 * @brief Scan named poses (SRDF <group_state>) and collect colliding link pairs.
 *
 * For each group_state matching the filter, sets a RobotState to that pose
 * (joints not listed default to the model's default values, mirroring upstream
 * MoveIt's "default pose" rule) and runs a self-collision check via the
 * PlanningScene already configured by the SRDF.
 *
 * @param srdf_config SRDF configuration providing the robot model, planning
 *                    scene, and group states.
 * @param pose_filter Names of <group_state>s to check. Empty means "all".
 * @throws std::runtime_error if pose_filter contains a name not present in the SRDF.
 * @return Colliding pairs across all matching poses (may contain duplicates
 *         across poses; caller deduplicates).
 */
std::vector<NamedPoseCollision>
collectNamedPoseCollisions( const std::shared_ptr<moveit_setup::SRDFConfig> &srdf_config,
                            const std::vector<std::string> &pose_filter );

/**
 * @brief Merge named-pose collisions into the SRDF's disabled-collisions set.
 *
 * Precedence: Adjacent > User > NamedPose > other reasons. Pairs already
 * disabled with reason "Adjacent", "User", or "NamedPose" are left untouched;
 * any other existing reason is overwritten to "NamedPose". Pairs not yet
 * disabled are appended.
 *
 * @return Number of pairs that were newly disabled (excluding relabels).
 */
unsigned int mergeNamedPoseCollisions( const std::shared_ptr<moveit_setup::SRDFConfig> &srdf_config,
                                       const std::vector<NamedPoseCollision> &collisions );

/**
 * @brief Snapshot user-authored <disable_collisions> entries (reason="User").
 *
 * Octomap entries are skipped — they are preserved separately. Pairs whose
 * link1 or link2 is not present in the robot model are dropped with a warning
 * (stale references should not survive a regeneration).
 *
 * Call this BEFORE updateCollisionMatrix() (which clears the disabled set).
 */
std::vector<srdf::Model::CollisionPair>
extractUserCollisions( const std::shared_ptr<moveit_setup::SRDFConfig> &srdf_config );

/**
 * @brief Re-merge previously-snapshotted user pairs into the disabled set.
 *
 * Precedence: Adjacent > User > everything else. A pair already disabled with
 * reason "Adjacent" is left as Adjacent (adjacency is structural). Any other
 * existing reason is relabeled to "User". Pairs not present are appended.
 *
 * @return Number of pairs that were newly added (excluding relabels).
 */
unsigned int restoreUserCollisions( const std::shared_ptr<moveit_setup::SRDFConfig> &srdf_config,
                                    const std::vector<srdf::Model::CollisionPair> &user_pairs );

} // namespace collision_matrix_updater

#endif // MOVEIT_COLLISION_MATRIX_UPDATER_MOVEIT_COLLISION_MATRIX_UPDATER_HPP
