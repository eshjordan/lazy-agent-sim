/**
Simulation of distributed coverage using lazy agents operating under discrete, local, event-triggered communication.
Copyright (C) 2024  Jordan Esh

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
**/

#include "lazy_agent_sim/LazyAgentSim.hpp"

#include <cstdio>
#include <rclcpp/executors.hpp>
#include <webots/motor.h>
#include <webots/robot.h>

constexpr auto HALF_DISTANCE_BETWEEN_WHEELS = 0.045;
constexpr auto WHEEL_RADIUS                 = 0.025;

namespace lazy_agent_sim {
void MyRobotDriver::init(webots_ros2_driver::WebotsNode *node,
                         std::unordered_map<std::string, std::string> & /*parameters*/)
{

    right_motor_ = wb_robot_get_device("right wheel motor");
    left_motor_  = wb_robot_get_device("left wheel motor");

    wb_motor_set_position(left_motor_, INFINITY);
    wb_motor_set_velocity(left_motor_, 0.0);

    wb_motor_set_position(right_motor_, INFINITY);
    wb_motor_set_velocity(right_motor_, 0.0);

    cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SensorDataQoS().reliable(), [this](const geometry_msgs::msg::Twist::ConstSharedPtr &msg) {
            this->cmd_vel_msg_.linear  = msg->linear;
            this->cmd_vel_msg_.angular = msg->angular;
        });
}

void MyRobotDriver::step()
{
    auto forward_speed = cmd_vel_msg_.linear.x;
    auto angular_speed = cmd_vel_msg_.angular.z;

    auto command_motor_left  = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS;
    auto command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS;

    wb_motor_set_velocity(left_motor_, command_motor_left);
    wb_motor_set_velocity(right_motor_, command_motor_right);
}
} // namespace lazy_agent_sim

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(lazy_agent_sim::MyRobotDriver, webots_ros2_driver::PluginInterface)
