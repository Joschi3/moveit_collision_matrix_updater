/*
 * Collision Matrix Updater (core implementation)
 */

#include <moveit_collision_matrix_updater/moveit_collision_matrix_updater.hpp>

#include <algorithm>
#include <stdexcept>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>

// MoveIt Setup Framework
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <moveit_setup_framework/data/urdf_config.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>

// Collision Computation
#include <moveit_setup_srdf_plugins/default_collisions.hpp>

// Self-collision check for named-pose scanning
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>

namespace collision_matrix_updater
{

using namespace moveit_setup;
using namespace moveit_setup::srdf_setup;

DataWarehousePtr initializeDataWarehouse( const rclcpp::Node::SharedPtr &parent_node,
                                          const std::string &urdf_path, const std::string &srdf_path )
{
  auto data_warehouse = std::make_shared<DataWarehouse>( parent_node );

  // Load URDF
  auto urdf_config = data_warehouse->get<URDFConfig>( "urdf" );
  urdf_config->loadFromPath( urdf_path );
  RCLCPP_INFO( parent_node->get_logger(), "URDF loaded from: %s", urdf_path.c_str() );

  // Load SRDF
  auto srdf_config = data_warehouse->get<SRDFConfig>( "srdf" );
  srdf_config->loadSRDFFile( srdf_path );
  RCLCPP_INFO( parent_node->get_logger(), "SRDF loaded from: %s", srdf_path.c_str() );

  return data_warehouse;
}

DefaultCollisions *initializeCollisionUpdater( const DataWarehousePtr &data_warehouse )
{
  auto collision_updater = new DefaultCollisions();

  // Create a minimal parent node for the collision updater
  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms( true );
  auto minimal_node = std::make_shared<rclcpp::Node>( "collision_updater", node_options );

  collision_updater->initialize( minimal_node, data_warehouse );

  RCLCPP_INFO( minimal_node->get_logger(), "DefaultCollisions initialized" );

  return collision_updater;
}

void updateCollisionMatrix( DefaultCollisions *collision_updater,
                            const std::shared_ptr<SRDFConfig> &srdf_config, unsigned int num_trials,
                            double min_collision_fraction )
{
  auto logger = rclcpp::get_logger( "collision_matrix_updater" );

  RCLCPP_INFO_STREAM( logger, "Starting collision matrix recalculation..." );
  RCLCPP_INFO_STREAM( logger, "  Trials: " << num_trials );
  RCLCPP_INFO_STREAM( logger, "  Min collision fraction: " << min_collision_fraction );

  // Clear existing collision data
  srdf_config->clearCollisionData();
  RCLCPP_INFO( logger, "Cleared existing collision data" );

  // Load current link pairs from SRDF (empty after clearing, but initializes structure)
  collision_updater->linkPairsFromSRDF();

  // Generate new collision table (blocking call)
  collision_updater->startGenerationThread( num_trials, min_collision_fraction );
  collision_updater->joinGenerationThread();

  // Write the updated collision pairs back to the SRDF
  collision_updater->linkPairsToSRDF();

  // Update the robot model to reflect changes
  srdf_config->updateRobotModel( COLLISIONS );

  RCLCPP_INFO( logger, "Collision matrix recalculation complete" );
  RCLCPP_INFO_STREAM(
      logger, "  Disabled collision pairs: " << srdf_config->getDisabledCollisions().size() );
}

void writeSRDFFile( const std::shared_ptr<SRDFConfig> &srdf_config, const std::string &output_path )
{
  auto logger = rclcpp::get_logger( "collision_matrix_updater" );

  if ( !srdf_config->write( output_path ) ) {
    throw std::runtime_error( "Failed to write SRDF file to: " + output_path );
  }

  RCLCPP_INFO_STREAM( logger, "SRDF file written to: " << output_path );
}

std::vector<NamedPoseCollision>
collectNamedPoseCollisions( const std::shared_ptr<SRDFConfig> &srdf_config,
                            const std::vector<std::string> &pose_filter )
{
  auto logger = rclcpp::get_logger( "collision_matrix_updater" );

  const auto &group_states = srdf_config->getGroupStates();
  if ( group_states.empty() ) {
    RCLCPP_WARN( logger, "No <group_state> entries found in SRDF; named-pose check is a no-op." );
    return {};
  }

  // Resolve which poses to scan. Empty filter == all.
  std::unordered_set<std::string> available;
  available.reserve( group_states.size() );
  for ( const auto &gs : group_states ) available.insert( gs.name_ );

  for ( const auto &requested : pose_filter ) {
    if ( available.find( requested ) == available.end() ) {
      throw std::runtime_error( "Named pose not found in SRDF <group_state>: '" + requested + "'" );
    }
  }
  const bool all_poses = pose_filter.empty();
  std::unordered_set<std::string> filter_set( pose_filter.begin(), pose_filter.end() );

  // Build a fresh PlanningScene from the (already-updated) robot model so the
  // ACM reflects the current <disable_collisions>. Pairs already disabled will
  // be filtered out by the ACM; we only see *new* colliding pairs.
  auto robot_model = srdf_config->getRobotModel();
  if ( !robot_model ) {
    throw std::runtime_error( "SRDFConfig has no robot model; cannot run named-pose check." );
  }
  auto planning_scene = std::make_shared<planning_scene::PlanningScene>( robot_model );

  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 1000;
  req.max_contacts_per_pair = 1;

  std::vector<NamedPoseCollision> results;

  for ( const auto &gs : group_states ) {
    if ( !all_poses && filter_set.find( gs.name_ ) == filter_set.end() )
      continue;

    moveit::core::RobotState state( robot_model );
    state.setToDefaultValues();

    for ( const auto &[joint_name, values] : gs.joint_values_ ) {
      if ( !robot_model->hasJointModel( joint_name ) ) {
        RCLCPP_WARN_STREAM( logger, "Pose '" << gs.name_ << "': joint '" << joint_name
                                             << "' not in robot model; skipping joint." );
        continue;
      }
      state.setJointPositions( joint_name, values );
    }
    state.update();

    collision_detection::CollisionResult res;
    planning_scene->checkSelfCollision( req, res, state );

    RCLCPP_INFO_STREAM( logger, "Pose '" << gs.name_ << "' (group '" << gs.group_
                                         << "'): " << res.contacts.size() << " colliding pair(s)" );

    for ( const auto &[link_pair, contacts] : res.contacts ) {
      (void)contacts;
      std::string a = link_pair.first;
      std::string b = link_pair.second;
      if ( a > b )
        std::swap( a, b );
      results.push_back( { a, b, gs.name_ } );
    }
  }

  return results;
}

unsigned int mergeNamedPoseCollisions( const std::shared_ptr<SRDFConfig> &srdf_config,
                                       const std::vector<NamedPoseCollision> &collisions )
{
  auto logger = rclcpp::get_logger( "collision_matrix_updater" );
  auto &disabled = srdf_config->getDisabledCollisions();

  // Build a lookup of currently-disabled pairs -> index in the vector.
  auto sorted_key = []( std::string a, std::string b ) {
    if ( a > b )
      std::swap( a, b );
    return std::make_pair( std::move( a ), std::move( b ) );
  };
  std::map<std::pair<std::string, std::string>, std::size_t> existing;
  for ( std::size_t i = 0; i < disabled.size(); ++i ) {
    existing.emplace( sorted_key( disabled[i].link1_, disabled[i].link2_ ), i );
  }

  // Deduplicate the incoming list (a pair may collide in multiple poses).
  std::set<std::pair<std::string, std::string>> unique_pairs;
  for ( const auto &c : collisions ) unique_pairs.emplace( sorted_key( c.link1, c.link2 ) );

  unsigned int newly_added = 0;
  for ( const auto &pair : unique_pairs ) {
    auto it = existing.find( pair );
    if ( it == existing.end() ) {
      srdf::Model::CollisionPair cp;
      cp.link1_ = pair.first;
      cp.link2_ = pair.second;
      cp.reason_ = "NamedPose";
      disabled.push_back( cp );
      ++newly_added;
    } else {
      auto &cp = disabled[it->second];
      if ( cp.reason_ == "Adjacent" || cp.reason_ == "NamedPose" )
        continue;
      cp.reason_ = "NamedPose";
    }
  }

  RCLCPP_INFO_STREAM( logger, "Named-pose merge: " << newly_added << " new pair(s) disabled, "
                                                   << unique_pairs.size() - newly_added
                                                   << " existing pair(s) relabeled or kept." );
  return newly_added;
}

} // namespace collision_matrix_updater
