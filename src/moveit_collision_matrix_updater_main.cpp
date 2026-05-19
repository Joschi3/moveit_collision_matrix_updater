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
 *   collision_matrix_updater <urdf_file> <srdf_file> [num_trials] [min_fraction]
 *                            [output_srdf_file] [--check-named-poses=all|n1,n2,...]
 *
 * Notes:
 *   - If [output_srdf_file] is omitted, the input SRDF path is used and the file is overwritten.
 *   - --check-named-poses scans <group_state> entries; colliding pairs are added with
 *     reason="NamedPose". Pass "all" or a comma-separated list of pose names.
 */

#include "moveit_collision_matrix_updater/moveit_collision_matrix_updater.hpp"

#include <filesystem>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit_setup_framework/data/srdf_config.hpp>

namespace fs = std::filesystem;

static void printUsage()
{
  std::cerr
      << "Usage:\n"
      << "  collision_matrix_updater <urdf_file> <srdf_file> [num_trials] [min_fraction] "
         "[output_srdf_file] [--check-named-poses=all|name1,name2,...]\n\n"
      << "Examples:\n"
      << "  collision_matrix_updater robot.urdf robot.srdf\n"
      << "  collision_matrix_updater robot.urdf robot.srdf 2000 0.02\n"
      << "  collision_matrix_updater robot.urdf robot_in.srdf 1000 0.05 robot_out.srdf\n"
      << "  collision_matrix_updater robot.urdf robot.srdf --check-named-poses=all\n"
      << "  collision_matrix_updater robot.urdf robot.srdf --check-named-poses=folded,front\n";
}

// Split "a,b,c" into {"a","b","c"}. Empty entries are skipped.
static std::vector<std::string> splitCsv( const std::string &s )
{
  std::vector<std::string> out;
  std::stringstream ss( s );
  std::string item;
  while ( std::getline( ss, item, ',' ) ) {
    if ( !item.empty() )
      out.push_back( item );
  }
  return out;
}

int main( int argc, char **argv )
{
  // Initialize ROS
  rclcpp::init( argc, argv );

  try {
    // Split argv into positional args and flag args. Flags are recognized as
    // tokens starting with "--"; everything else stays positional, in order.
    std::vector<std::string> positional;
    std::optional<std::string> named_poses_arg;
    for ( int i = 1; i < argc; ++i ) {
      std::string a = argv[i];
      const std::string flag = "--check-named-poses";
      if ( a.rfind( flag, 0 ) == 0 ) {
        if ( a.size() > flag.size() && a[flag.size()] == '=' ) {
          named_poses_arg = a.substr( flag.size() + 1 );
        } else if ( i + 1 < argc ) {
          named_poses_arg = argv[++i];
        } else {
          std::cerr << "Error: --check-named-poses requires a value (all or name1,name2,...).\n";
          printUsage();
          return 1;
        }
      } else if ( a.rfind( "--", 0 ) == 0 ) {
        std::cerr << "Error: unknown flag '" << a << "'.\n";
        printUsage();
        return 1;
      } else {
        positional.push_back( std::move( a ) );
      }
    }

    if ( positional.size() < 2 ) {
      printUsage();
      return 1;
    }

    std::string urdf_path = positional[0];
    std::string srdf_input_path = positional[1];

    unsigned int num_trials = 50000;
    double min_fraction = 0.95;
    std::string srdf_output_path = srdf_input_path; // default: overwrite input

    if ( positional.size() > 2 )
      num_trials = std::stoul( positional[2] );
    if ( positional.size() > 3 )
      min_fraction = std::stod( positional[3] );
    if ( positional.size() > 4 )
      srdf_output_path = positional[4];

    // Resolve named-pose filter ("all" => empty list = "all poses").
    std::vector<std::string> named_pose_filter;
    bool run_named_pose_check = false;
    if ( named_poses_arg.has_value() ) {
      run_named_pose_check = true;
      if ( *named_poses_arg != "all" ) {
        named_pose_filter = splitCsv( *named_poses_arg );
        if ( named_pose_filter.empty() ) {
          std::cerr << "Error: --check-named-poses value is empty.\n";
          return 1;
        }
      }
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

    // Optionally scan named <group_state> poses for additional collisions
    if ( run_named_pose_check ) {
      RCLCPP_INFO_STREAM( logger, "Scanning named poses for collisions"
                                      << ( named_pose_filter.empty() ? " (all)"
                                                                     : std::string( " (filter: " ) +
                                                                           *named_poses_arg + ")" ) );
      auto named_pose_collisions =
          collision_matrix_updater::collectNamedPoseCollisions( srdf_config, named_pose_filter );
      const unsigned int newly_added =
          collision_matrix_updater::mergeNamedPoseCollisions( srdf_config, named_pose_collisions );
      RCLCPP_INFO_STREAM( logger, "Named-pose scan: " << newly_added
                                                      << " new disable_collisions entries added." );
      // Refresh the model so subsequent steps see the updated ACM.
      srdf_config->updateRobotModel( moveit_setup::COLLISIONS );
    }

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
