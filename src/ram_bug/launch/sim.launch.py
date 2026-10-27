"""
Gazebo (Harmonic) test: 3 x 3 m arena + our TurtleBot3 Burger + a trap chaser.

  ros2 launch ram_bug sim.launch.py                      # world arena_1
  ros2 launch ram_bug sim.launch.py world:=arena_3 chaser_mode:=pursuit
  ros2 launch ram_bug sim.launch.py share:=false              # no position sharing: LiDAR only
  ros2 launch ram_bug sim.launch.py goal_x:=2.5 goal_y:=2.7   # override the goal
  ros2 launch ram_bug sim.launch.py goal_x:=nan               # wait for RViz 2D Goal Pose
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory("ram_bug")
    tb3 = get_package_share_directory("turtlebot3_gazebo")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    world = context.launch_configurations["world"]
    sdf = os.path.join(share, "worlds", world + ".sdf")
    with open(os.path.join(share, "worlds", world + ".json"), encoding="utf-8") as f:
        info = json.load(f)
    gx = context.launch_configurations["goal_x"]
    gy = context.launch_configurations["goal_y"]
    goal_x = info["goal_x"] if gx == "" else float(gx)
    goal_y = info["goal_y"] if gy == "" else float(gy)
    sx, sy = info["start_x"], info["start_y"]
    use_sim_time = True

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v2 {sdf}", "on_exit_shutdown": "true"}.items())
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3, "launch", "robot_state_publisher.launch.py")),
        launch_arguments={"use_sim_time": "true"}.items())
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3, "launch", "spawn_turtlebot3.launch.py")),
        launch_arguments={"x_pose": str(sx), "y_pose": str(sy)}.items())
    chaser_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="chaser_bridge", output="screen",
        arguments=["/model/chaser/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                   "/chaser/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"])
    arena_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="arena_to_odom",
        arguments=["--x", str(sx), "--y", str(sy), "--yaw", "0.0", "--frame-id", "arena", "--child-frame-id", "odom"],
        parameters=[{"use_sim_time": use_sim_time}])
    ram_bug = Node(
        package="ram_bug", executable="ram_bug_node", name="ram_bug", output="screen",
        parameters=[os.path.join(share, "config", "ram_bug.yaml"),
                    {"use_sim_time": use_sim_time, "start_x": sx, "start_y": sy, "start_yaw": 0.0,
                     "goal_x": goal_x, "goal_y": goal_y, "cmd_vel_stamped": "true",
                     "algorithm": context.launch_configurations["algorithm"],
                     "drive": context.launch_configurations["drive"]}])
    chaser = Node(
        package="ram_bug", executable="trap_chaser_node", name="trap_chaser", output="screen",
        parameters=[{"use_sim_time": use_sim_time, "world_json": os.path.join(share, "worlds", world + ".json"),
                     "goal_x": goal_x if goal_x == goal_x else info["goal_x"],
                     "goal_y": goal_y if goal_y == goal_y else info["goal_y"],
                     "start_x": sx, "start_y": sy, "start_yaw": 0.0,
                     "mode": context.launch_configurations["chaser_mode"]}])
    peer = Node(
        package="ram_bug", executable="peer_link_node", name="peer_link", output="screen",
        parameters=[{"use_sim_time": use_sim_time, "peer_host": "", "listen_port": 47002,
                     "start_x": sx, "start_y": sy, "start_yaw": 0.0, "peer_frame": "arena"}])
    out = [gz, rsp, spawn, chaser_bridge, arena_tf, ram_bug, chaser]
    if context.launch_configurations["share"] == "true":
        out.append(peer)
    if context.launch_configurations["rviz"] == "true":
        out.append(Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(share, "config", "ram_bug.rviz")],
                        parameters=[{"use_sim_time": use_sim_time}]))
    return out


def generate_launch_description():
    tb3 = get_package_share_directory("turtlebot3_gazebo")
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="arena_1"),
        DeclareLaunchArgument("goal_x", default_value="", description="arena metres; empty = from world json"),
        DeclareLaunchArgument("goal_y", default_value=""),
        DeclareLaunchArgument("chaser_mode", default_value="friend", description="friend, trap or pursuit"),
        DeclareLaunchArgument("share", default_value="true", description="share positions over UDP (like the real match)"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("algorithm", default_value="bug2", description="bug2 or ram_bug"),
        DeclareLaunchArgument("drive", default_value="smooth", description="smooth or spin"),
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(tb3, "models")),
        OpaqueFunction(function=setup),
    ])
