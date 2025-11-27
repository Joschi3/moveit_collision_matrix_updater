# Collision Matrix Updater

A utility that recalculates the collision matrix for a robot's SRDF file without needing to rerun the MoveIt Setup Assistant.

## Overview

The collision matrix is a critical component of robot motion planning that defines which link pairs should or should not be checked for collisions. Typically, you would regenerate this using the MoveIt Setup Assistant GUI, which is time-consuming. This utility provides a command-line tool to update the collision matrix directly.

## How It Works

The collision matrix updater performs the following steps:

1. **Load URDF**: Parses the robot's URDF (Unified Robot Description Format) file
2. **Load SRDF**: Parses the robot's SRDF (Semantic Robot Description Format) file
3. **Initialize DefaultCollisions**: Sets up the collision computation engine
4. **Compute Default Collisions**: Runs sampling-based collision detection to determine which link pairs are:
    - Never in collision (can always check)
    - Always in collision (should never check)
    - Adjacent links (default disabled)
5. **Update SRDF**: Writes the new collision pairs back to the SRDF file

## Usage

### Basic Usage

```bash
collision_matrix_updater <urdf_file> <srdf_file>
```

### Advanced Usage with Custom Parameters

```bash
collision_matrix_updater <urdf_file> <srdf_file> [num_trials] [min_fraction]
```

### Parameters

- `urdf_file` (required): Path to the robot's URDF file
- `srdf_file` (required): Path to the robot's SRDF file (will be updated in-place)
- `num_trials` (optional): Number of random collision checks (default: 1000)
    - Higher values increase accuracy but take longer
    - Recommended: 1000-10000 for most robots
- `min_fraction` (optional): Minimum collision fraction threshold (default: 0.05)
    - Link pairs with collision frequency >= this value are marked as "ALWAYS in collision"
    - Range: 0.0 - 1.0
    - Lower values = more pairs marked as always colliding

### Examples

```bash
# Basic usage with defaults (1000 trials, 5% threshold)
collision_matrix_updater my_robot.urdf my_robot.srdf

# High precision (more trials, more accurate but slower)
collision_matrix_updater my_robot.urdf my_robot.srdf 5000 0.05

# Quick estimation (fewer trials, faster but less accurate)
collision_matrix_updater my_robot.urdf my_robot.srdf 500 0.1
```

## Implementation Details

### Key Classes Used

#### SRDFConfig
- **Purpose**: Manages SRDF loading, updating, and writing
- **Key Methods**:
    - `loadSRDFFile()`: Loads SRDF from disk
    - `clearCollisionData()`: Clears existing collision pairs
    - `getDisabledCollisions()`: Access the collision pair list
    - `updateRobotModel()`: Updates internal robot model
    - `write()`: Writes SRDF back to disk

#### DefaultCollisions
- **Purpose**: Orchestrates collision matrix computation
- **Key Methods**:
    - `linkPairsFromSRDF()`: Loads collision pairs from SRDF
    - `generateCollisionTable()`: Computes new collision matrix via sampling
    - `linkPairsToSRDF()`: Saves computed pairs back to SRDF

#### DataWarehouse
- **Purpose**: Container for all configuration singletons
- **Usage**: Centralizes access to URDF and SRDF configurations

### Collision Computation Algorithm

The `computeDefaultCollisions()` function:

1. Generates all possible link pairs (n choose 2)
2. Randomly samples robot configurations
3. For each sample, checks which link pairs are in collision
4. Classifies pairs into categories:
    - **NEVER**: No collisions found in any sample
    - **ALWAYS**: Collisions found in >= `min_fraction` samples
    - **DEFAULT**: Collisions found in some samples but < `min_fraction`
    - **ADJACENT**: Links connected by a fixed joint
    - **USER**: Manually disabled by user

## Building from Source

### Prerequisites

- ROS 2 (Humble or later)
- MoveIt 2
- Colcon

### Build Steps

1. Place `collision_matrix_updater.cpp` in your workspace
2. Create a CMakeLists.txt (see CMakeLists_collision_updater.txt)
3. Build the package:

```bash
cd /path/to/workspace
colcon build --packages-select collision_matrix_updater
```

4. Source the setup:

```bash
source install/setup.bash
```

5. Run:

```bash
ros2 run collision_matrix_updater collision_matrix_updater my_robot.urdf my_robot.srdf
```

## Code Example: Using Programmatically

If you want to integrate the collision matrix updater into your own code:

```cpp
#include <moveit_setup_framework/data_warehouse.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <moveit_setup_srdf_plugins/default_collisions.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace moveit_setup;
using namespace moveit_setup::srdf_setup;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("my_updater");
  auto data_warehouse = std::make_shared<DataWarehouse>(node);
  
  // Load configurations
  auto urdf_config = data_warehouse->get<URDFConfig>("urdf");
  urdf_config->loadURDFFile("my_robot.urdf");
  
  auto srdf_config = data_warehouse->get<SRDFConfig>("srdf");
  srdf_config->loadSRDFFile("my_robot.srdf");
  
  // Initialize collision updater
  auto collision_updater = std::make_shared<DefaultCollisions>();
  collision_updater->initialize(node, data_warehouse);
  
  // Update collision matrix
  srdf_config->clearCollisionData();
  collision_updater->linkPairsFromSRDF();
  collision_updater->generateCollisionTable(1000, 0.05, true);
  collision_updater->linkPairsToSRDF();
  srdf_config->updateRobotModel(COLLISIONS);
  
  // Write result
  srdf_config->write("my_robot_updated.srdf");
  
  rclcpp::shutdown();
  return 0;
}
```

## Troubleshooting

### Error: "SRDF file not found"
- Verify the SRDF file path is correct
- Use absolute paths or relative paths from current working directory

### Error: "URDF file not found"
- Verify the URDF file path is correct
- Make sure URDF is specified before SRDF

### Very slow computation
- Reduce `num_trials` parameter (default 1000 is usually good)
- The computation is CPU-intensive; this is normal
- Run on a machine with good CPU performance

### Results not as expected
- Increase `num_trials` for better accuracy
- Verify the URDF and SRDF are valid
- Check that robot has collision geometry defined

## Performance Notes

- Computation time increases linearly with `num_trials`
- Typical timings on modern CPU:
    - 500 trials: ~30 seconds
    - 1000 trials: ~60 seconds
    - 5000 trials: ~5 minutes
- Memory usage is minimal (<100MB typically)

## Backing Up Your SRDF

Before running the updater, it's recommended to back up your SRDF file:

```bash
cp my_robot.srdf my_robot.srdf.backup
collision_matrix_updater my_robot.urdf my_robot.srdf
```

If the results aren't satisfactory, you can restore the backup:

```bash
cp my_robot.srdf.backup my_robot.srdf
```

## References

- [MoveIt Setup Assistant Documentation](https://docs.ros.org/en/humble/Tutorials/Advanced/Moveit/Setup-Assistant/Setup-Assistant-Intro.html)
- [SRDF Format](http://wiki.ros.org/srdf/XML)
- [Collision Detection in MoveIt](https://docs.ros.org/en/humble/Tutorials/Advanced/Moveit/Motion-Planning-Pipeline/Motion-Planning-Pipeline.html)
