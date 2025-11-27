/*
 * Collision Matrix Updater
 * 
 * This utility recalculates the collision matrix for a robot's SRDF without
 * needing to rerun the MoveIt Setup Assistant.
 * 
 * Usage:
 *   collision_matrix_updater <urdf_file> <srdf_file> [num_trials] [min_fraction]
 * 
 * Example:
 *   collision_matrix_updater robot.urdf robot.srdf 1000 0.05
 */

#include <iostream>
#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

// MoveIt Core
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <urdf/model.h>
#include <srdfdom/srdf_writer.h>

// MoveIt Setup Framework
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>
#include <moveit_setup_framework/data/urdf_config.hpp>

// Collision Computation
#include <moveit_setup_srdf_plugins/default_collisions.hpp>
#include <moveit_setup_srdf_plugins/compute_default_collisions.hpp>

namespace fs = std::filesystem;
using namespace moveit_setup;
using namespace moveit_setup::srdf_setup;

/**
 * @brief Initialize the DataWarehouse with URDF and SRDF configurations
 * @param parent_node The ROS2 node
 * @param urdf_path Path to the URDF file
 * @param srdf_path Path to the SRDF file
 * @return Configured DataWarehouse
 */
DataWarehousePtr initializeDataWarehouse(const rclcpp::Node::SharedPtr& parent_node,
                                         const std::string& urdf_path,
                                         const std::string& srdf_path)
{
  auto data_warehouse = std::make_shared<DataWarehouse>(parent_node);
  
  // Load URDF
  auto urdf_config = data_warehouse->get<URDFConfig>("urdf");
  urdf_config->loadFromPath( urdf_path );
  RCLCPP_INFO(parent_node->get_logger(), "URDF loaded from: %s", urdf_path.c_str());

  // Load SRDF
  auto srdf_config = data_warehouse->get<SRDFConfig>("srdf");
  srdf_config->loadSRDFFile(srdf_path);
  RCLCPP_INFO(parent_node->get_logger(), "SRDF loaded from: %s", srdf_path.c_str());

  return data_warehouse;
}

/**
 * @brief Initialize the DefaultCollisions updater
 * @param data_warehouse The configured data warehouse
 * @return DefaultCollisions instance
 */
DefaultCollisions* initializeCollisionUpdater(const DataWarehousePtr& data_warehouse)
{
  auto collision_updater = new DefaultCollisions();
  
  // Create a minimal parent node for the collision updater
  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);
  auto minimal_node = std::make_shared<rclcpp::Node>("collision_updater", node_options);
  
  collision_updater->initialize(minimal_node, data_warehouse);
  
  RCLCPP_INFO(minimal_node->get_logger(), "DefaultCollisions initialized");
  
  return collision_updater;
}



/**
 * @brief Recalculate and update the collision matrix in the SRDF
 * @param collision_updater The DefaultCollisions instance
 * @param srdf_config The SRDF configuration
 * @param num_trials Number of collision checking trials
 * @param min_collision_fraction Minimum collision fraction threshold
 */
void updateCollisionMatrix(DefaultCollisions* collision_updater,
                          const std::shared_ptr<SRDFConfig>& srdf_config,
                          unsigned int num_trials = 1000,
                          double min_collision_fraction = 0.05)
{
  auto logger = rclcpp::get_logger("collision_matrix_updater");
  
  RCLCPP_INFO_STREAM(logger, "Starting collision matrix recalculation...");
  RCLCPP_INFO_STREAM(logger, "  Trials: " << num_trials);
  RCLCPP_INFO_STREAM(logger, "  Min collision fraction: " << min_collision_fraction);

  // Clear existing collision data
  srdf_config->clearCollisionData();
  RCLCPP_INFO(logger, "Cleared existing collision data");

  // Load current link pairs from SRDF (empty after clearing, but initializes structure)
  collision_updater->linkPairsFromSRDF();

  // Generate new collision table (blocking call)
  collision_updater->startGenerationThread( num_trials, min_collision_fraction );
  collision_updater->joinGenerationThread();

  // Write the updated collision pairs back to the SRDF
  collision_updater->linkPairsToSRDF();

  // Update the robot model to reflect changes
  srdf_config->updateRobotModel(COLLISIONS);

  RCLCPP_INFO(logger, "Collision matrix recalculation complete");
  RCLCPP_INFO_STREAM(logger, "  Disabled collision pairs: " 
                             << srdf_config->getDisabledCollisions().size());
}

/**
 * @brief Write the updated SRDF file
 * @param srdf_config The SRDF configuration
 * @param output_path Path where the SRDF file will be written
 */
void writeSRDFFile(const std::shared_ptr<SRDFConfig>& srdf_config,
                   const std::string& output_path)
{
  auto logger = rclcpp::get_logger("collision_matrix_updater");
  
  if (!srdf_config->write(output_path))
  {
    throw std::runtime_error("Failed to write SRDF file to: " + output_path);
  }

  RCLCPP_INFO_STREAM(logger, "SRDF file written to: " << output_path);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  
  try
  {
    // Parse command line arguments
    if (argc < 3)
    {
      std::cerr << "Usage: collision_matrix_updater <urdf_file> <srdf_file> [num_trials] [min_fraction]\n"
                << "Example: collision_matrix_updater robot.urdf robot.srdf 1000 0.05\n";
      return 1;
    }

    std::string urdf_path = argv[1];
    std::string srdf_path = argv[2];
    unsigned int num_trials = (argc > 3) ? std::stoul(argv[3]) : 1000;
    double min_fraction = (argc > 4) ? std::stod(argv[4]) : 0.05;

    // Verify files exist
    if (!fs::exists(urdf_path))
    {
      throw std::runtime_error("URDF file not found: " + urdf_path);
    }
    if (!fs::exists(srdf_path))
    {
      throw std::runtime_error("SRDF file not found: " + srdf_path);
    }

    // Create ROS node
    rclcpp::NodeOptions node_options;
    node_options.use_intra_process_comms(true);
    auto node = std::make_shared<rclcpp::Node>("collision_matrix_updater", node_options);

    auto logger = node->get_logger();
    RCLCPP_INFO_STREAM(logger, "Collision Matrix Updater");
    RCLCPP_INFO_STREAM(logger, "URDF: " << urdf_path);
    RCLCPP_INFO_STREAM(logger, "SRDF: " << srdf_path);

    // Initialize data warehouse with URDF and SRDF
    auto data_warehouse = initializeDataWarehouse(node, urdf_path, srdf_path);

    // Get SRDF config
    auto srdf_config = data_warehouse->get<SRDFConfig>("srdf");

    // Initialize collision updater
    auto collision_updater = initializeCollisionUpdater(data_warehouse);

    // Recalculate collision matrix
    updateCollisionMatrix(collision_updater, srdf_config, num_trials, min_fraction);

    // Write updated SRDF file
    writeSRDFFile(srdf_config, srdf_path);

    RCLCPP_INFO(logger, "Done! Collision matrix has been updated in the SRDF file.");

    delete collision_updater;
    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("collision_matrix_updater"), 
                        "Error: " << e.what());
    rclcpp::shutdown();
    return 1;
  }
}
