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

"""Launch Webots."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    """Launch Webots."""
    package_dir = get_package_share_directory("lazy_agent_sim")
    webots = WebotsLauncher(
        world=os.path.join(package_dir, "worlds", "e-puck2_server.wbt")
    )
    return LaunchDescription([webots])
