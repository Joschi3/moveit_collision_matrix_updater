//
// Created by aljoscha-schmidt on 11/27/25.
//
/*
 * Collision Matrix Updater (CLI / main)
 *
 * This utility recalculates the collision matrix for a robot's SRDF without
 * needing to rerun the MoveIt Setup Assistant.
 *
 * Usage:
 *   collision_matrix_updater <urdf_file> <srdf_file> [num_trials] [min_fraction] [output_srdf_file]
 *
 * Notes:
 *   - If [output_srdf_file] is omitted, the input SRDF path is used and the file is overwritten.
 *   - num_trials defaults to 1000, min_fraction defaults to 0.05.
 */

#include "moveit_collision_matrix_updater/moveit_collision_matrix_updater.hpp"

#include <filesystem>
#include <iostream>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit_setup_framework/data/srdf_config.hpp>

namespace fs = std::filesystem;

static void printUsage()
{
  std::cerr << "Usage:\n"
            << "  collision_matrix_updater <urdf_file> <srdf_file> [num_trials] [min_fraction] "
               "[output_srdf_file]\n\n"
            << "Examples:\n"
            << "  collision_matrix_updater robot.urdf robot.srdf\n"
            << "  collision_matrix_updater robot.urdf robot.srdf 2000 0.02\n"
            << "  collision_matrix_updater robot.urdf robot_in.srdf 1000 0.05 robot_out.srdf\n";
}

int main( int argc, char **argv )
{
  // Initialize ROS
  rclcpp::init( argc, argv );

  try {
    if ( argc < 3 ) {
      printUsage();
      return 1;
    }

    std::string urdf_path = argv[1];
    std::string srdf_input_path = argv[2];

    unsigned int num_trials = 1000;
    double min_fraction = 0.05;
    std::string srdf_output_path = srdf_input_path; // default: overwrite input

    if ( argc > 3 ) {
      num_trials = std::stoul( argv[3] );
    }
    if ( argc > 4 ) {
      min_fraction = std::stod( argv[4] );
    }
    if ( argc > 5 ) {
      srdf_output_path = argv[5];
    }

    // Verify input files exist
    if ( !fs::exists( urdf_path ) ) {
      throw std::runtime_error( "URDF file not found: " + urdf_path );
    }
    if ( !fs::exists( srdf_input_path ) ) {
      throw std::runtime_error( "SRDF file not found: " + srdf_input_path );
    }

    // Optionally, check that output directory exists (if a directory is specified)
    const fs::path out_path( srdf_output_path );
    if ( !out_path.parent_path().empty() && !fs::exists( out_path.parent_path() ) ) {
      throw std::runtime_error( "Output directory does not exist: " +
                                out_path.parent_path().string() );
    }

    // Create ROS node
    rclcpp::NodeOptions node_options;
    node_options.use_intra_process_comms( true );
    auto node = std::make_shared<rclcpp::Node>( "collision_matrix_updater", node_options );

    auto logger = node->get_logger();
    RCLCPP_INFO_STREAM( logger, "Collision Matrix Updater" );
    RCLCPP_INFO_STREAM( logger, "URDF: " << urdf_path );
    RCLCPP_INFO_STREAM( logger, "SRDF (input): " << srdf_input_path );
    RCLCPP_INFO_STREAM( logger, "SRDF (output): " << srdf_output_path );
    RCLCPP_INFO_STREAM( logger, "Trials: " << num_trials );
    RCLCPP_INFO_STREAM( logger, "Min collision fraction: " << min_fraction );

    // Initialize data warehouse with URDF and SRDF
    auto data_warehouse =
        collision_matrix_updater::initializeDataWarehouse( node, urdf_path, srdf_input_path );

    // Get SRDF config
    auto srdf_config = data_warehouse->get<moveit_setup::SRDFConfig>( "srdf" );

    // Extract existing octomap collision pairs
    auto coll_pairs = srdf_config->getDisabledCollisions();
    std::vector<srdf::Model::CollisionPair> octomap_collisions;
    for ( const auto &pair : coll_pairs ) {
      if ( pair.link1_ == "<octomap>" || pair.link2_ == "<octomap>" ) {
        octomap_collisions.push_back( pair );
      }
    }

    // Initialize collision updater
    auto *collision_updater = collision_matrix_updater::initializeCollisionUpdater( data_warehouse );

    // Recalculate collision matrix
    collision_matrix_updater::updateCollisionMatrix( collision_updater, srdf_config, num_trials,
                                                     min_fraction );

    // Restore octomap collision pairs
    for ( const auto &pair : octomap_collisions ) {
      srdf_config->getDisabledCollisions().push_back( pair );
      RCLCPP_INFO_STREAM( logger, "Adding Collision Pair with Octomap: " << pair.link1_ << " - "
                                                                         << pair.link2_ );
    }

    // Write updated SRDF file (to input or separate output path)
    collision_matrix_updater::writeSRDFFile( srdf_config, srdf_output_path );

    RCLCPP_INFO( logger, "Done! Collision matrix has been updated in the SRDF file." );

    delete collision_updater;
    rclcpp::shutdown();
    return 0;
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR_STREAM( rclcpp::get_logger( "collision_matrix_updater" ), "Error: " << e.what() );
    rclcpp::shutdown();
    return 1;
  }
}
