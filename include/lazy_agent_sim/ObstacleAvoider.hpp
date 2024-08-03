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

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/range.hpp>

class ObstacleAvoider : public rclcpp::Node
{
public:
    explicit ObstacleAvoider();

private:
    void LeftSensorCallback(const sensor_msgs::msg::Range::ConstSharedPtr &msg);
    void RightSensorCallback(const sensor_msgs::msg::Range::ConstSharedPtr &msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sensor_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sensor_sub_;

    double left_sensor_value_{0.0};
    double right_sensor_value_{0.0};
};
