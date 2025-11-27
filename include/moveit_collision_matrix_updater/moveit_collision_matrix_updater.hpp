#ifndef MOVEIT_COLLISION_MATRIX_UPDATER_MOVEIT_COLLISION_MATRIX_UPDATER_HPP
#define MOVEIT_COLLISION_MATRIX_UPDATER_MOVEIT_COLLISION_MATRIX_UPDATER_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>


namespace moveit_collision_matrix_updater
{

class MoveitCollisionMatrixUpdater : public rclcpp::Node
{
public:

  MoveitCollisionMatrixUpdater();

private:
  //! @brief Sets up subscribers, publishers, etc. to configure the node
  void setup();

private:

};

}

#endif // MOVEIT_COLLISION_MATRIX_UPDATER_MOVEIT_COLLISION_MATRIX_UPDATER_HPP
