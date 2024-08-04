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

#include "lazy_agent_sim/ObstacleAvoider.hpp"
#include <rclcpp/executors.hpp>

constexpr auto MAX_RANGE = 0.15;

ObstacleAvoider::ObstacleAvoider() : Node("obstacle_avoider")
{
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    left_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
        "/left_sensor", 1,
        [this](const sensor_msgs::msg::Range::ConstSharedPtr &msg) { this->LeftSensorCallback(msg); });

    right_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
        "/right_sensor", 1,
        [this](const sensor_msgs::msg::Range::ConstSharedPtr &msg) { this->RightSensorCallback(msg); });
}

void ObstacleAvoider::LeftSensorCallback(const sensor_msgs::msg::Range::ConstSharedPtr &msg)
{
    left_sensor_value_ = msg->range;
}

void ObstacleAvoider::RightSensorCallback(const sensor_msgs::msg::Range::ConstSharedPtr &msg)
{
    right_sensor_value_ = msg->range;

    auto command_message = std::make_unique<geometry_msgs::msg::Twist>();

    command_message->linear.x = 0.1;

    if (left_sensor_value_ < 0.9 * MAX_RANGE || right_sensor_value_ < 0.9 * MAX_RANGE)
    {
        command_message->angular.z = -2.0;
    }

    publisher_->publish(std::move(command_message));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto avoider = std::make_shared<ObstacleAvoider>();
    rclcpp::spin(avoider);
    rclcpp::shutdown();
    return 0;
}
