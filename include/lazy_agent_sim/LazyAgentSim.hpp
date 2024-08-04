/**
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

#pragma once

#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"

namespace lazy_agent_sim {
class MyRobotDriver : public webots_ros2_driver::PluginInterface
{
public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    geometry_msgs::msg::Twist cmd_vel_msg_;

    WbDeviceTag right_motor_ = 0;
    WbDeviceTag left_motor_  = 0;
};
} // namespace lazy_agent_sim
