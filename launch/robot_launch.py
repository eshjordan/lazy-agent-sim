# Copyright (C) 2024  Jordan Esh.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def get_ros2_control_spawners(*args):
    # Declare here all nodes that must be restarted at simulation reset
    ros_control_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffdrive_controller"],
    )
    return [ros_control_node]


def generate_launch_description():
    package_dir = get_package_share_directory("lazy_agent_sim")
    robot_description_path = os.path.join(package_dir, "resource", "my_robot.urdf")

    webots_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lazy_agent_sim"),
                "launch",
                "bringup_launch.py",
            )
        ),
    )

    my_robot_driver = WebotsController(
        robot_name="my_robot",
        parameters=[
            {"robot_description": robot_description_path},
        ],
        # Every time one resets the simulation the controller is automatically respawned
        respawn=True,
    )

    obstacle_avoider = Node(
        package="lazy_agent_sim",
        executable="obstacle_avoider",
        respawn=True,
    )

    # Declare the reset handler that respawns nodes when robot_driver exits
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=my_robot_driver,
            on_exit=get_ros2_control_spawners,
        )
    )

    # Declare the handler that shuts all nodes down when robot_driver exits
    shutdown_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=my_robot_driver,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    return LaunchDescription(
        [
            webots_node,
            my_robot_driver,
            obstacle_avoider,
            # reset_handler,
            shutdown_handler,
        ]
        + get_ros2_control_spawners()
    )
