/*
 * Collision Matrix Updater (core implementation)
 */

#include <moveit_collision_matrix_updater/moveit_collision_matrix_updater.hpp>

#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

// MoveIt Setup Framework
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <moveit_setup_framework/data/urdf_config.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>

// Collision Computation
#include <moveit_setup_srdf_plugins/default_collisions.hpp>

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

} // namespace collision_matrix_updater
