"""
Run RAM-Bug on the REAL TurtleBot3 Burger.

First, on the robot (SBC):   ros2 launch turtlebot3_bringup robot.launch.py
Then (robot or remote PC, same ROS_DOMAIN_ID):

  ros2 launch ram_bug robot.launch.py start_x:=0.15 start_y:=0.15 start_yaw:=0.0 goal_x:=2.5 goal_y:=2.7

All numbers are metres / radians in the ARENA frame (origin = arena corner).
start_yaw = direction the robot faces at the start (0 = along the arena x axis,
1.5708 = along the y axis). Leave goal_x/goal_y out to send the goal from RViz.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory("ram_bug")
    lc = context.launch_configurations
    sx, sy, syaw = float(lc["start_x"]), float(lc["start_y"]), float(lc["start_yaw"])
    out = [
        Node(package="tf2_ros", executable="static_transform_publisher", name="arena_to_odom",
             arguments=["--x", str(sx), "--y", str(sy), "--yaw", str(syaw),
                        "--frame-id", "arena", "--child-frame-id", "odom"]),
        Node(package="ram_bug", executable="ram_bug_node", name="ram_bug", output="screen",
             parameters=[os.path.join(share, "config", "ram_bug.yaml"),
                         {"start_x": sx, "start_y": sy, "start_yaw": syaw,
                          "goal_x": float(lc["goal_x"]), "goal_y": float(lc["goal_y"]),
                          "capture_r": float(lc["capture_r"]),
                          "cmd_vel_stamped": lc["cmd_vel_stamped"]}]),
    ]
    if lc["rviz"] == "true":
        out.append(Node(package="rviz2", executable="rviz2",
                        arguments=["-d", os.path.join(share, "config", "ram_bug.rviz")]))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("start_x", default_value="0.15"),
        DeclareLaunchArgument("start_y", default_value="0.15"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("goal_x", default_value="nan"),
        DeclareLaunchArgument("goal_y", default_value="nan"),
        DeclareLaunchArgument("capture_r", default_value="0.30"),
        DeclareLaunchArgument("cmd_vel_stamped", default_value="auto"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=setup),
    ])
