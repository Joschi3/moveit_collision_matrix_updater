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

#include <rclcpp/rclcpp.hpp>

// MoveIt Setup Framework
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>

// Collision Computation
#include <moveit_setup_srdf_plugins/default_collisions.hpp>

namespace collision_matrix_updater
{

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

} // namespace collision_matrix_updater

#endif // MOVEIT_COLLISION_MATRIX_UPDATER_MOVEIT_COLLISION_MATRIX_UPDATER_HPP
