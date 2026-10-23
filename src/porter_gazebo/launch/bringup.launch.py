"""Start the warehouse, robot, shared bridge, RViz and optional demo nodes."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess
from launch.actions import LogError, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def _setup(context):
    """Resolve installed resources and build an event-driven startup sequence."""
    gazebo_share = Path(get_package_share_directory('porter_gazebo'))
    description_share = Path(get_package_share_directory('porter_description'))
    control_share = Path(get_package_share_directory('porter_control'))
    robot_description = xacro.process_file(
        str(description_share / 'urdf/porter.urdf.xacro')).toxml()
    gui_enabled = IfCondition(LaunchConfiguration('gui')).evaluate(context)
    rviz_enabled = IfCondition(LaunchConfiguration('rviz')).evaluate(context)
    behaviour_enabled = IfCondition(LaunchConfiguration('behaviour')).evaluate(context)
    walker_enabled = IfCondition(LaunchConfiguration('walker')).evaluate(context)
    follow_enabled = IfCondition(LaunchConfiguration('follow')).evaluate(context)
    models = str(gazebo_share / 'models')
    existing_resources = context.environment.get('GZ_SIM_RESOURCE_PATH', '')
    resources = os.pathsep.join(p for p in (models, existing_resources) if p)

    server_command = ['gz', 'sim', '-s', '-r', str(gazebo_share / 'worlds/porter_world.sdf')]
    if not gui_enabled:
        server_command.append('--headless-rendering')
    server = ExecuteProcess(
        cmd=server_command,
        name='gazebo_server', output='screen')
    gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '--gui-config', str(gazebo_share / 'config/gui.config')],
        name='gazebo_gui', output='screen')
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str),
                     'use_sim_time': True}],
        output='screen')
    # create waits for Gazebo's service and the transient-local description.
    # Spawn on clear ground: further back would overlap the loading deck.
    spawn = Node(
        package='ros_gz_sim', executable='create', name='spawn_porter',
        arguments=['-world', 'porter_world', '-topic', '/robot_description',
                   '-name', 'porter', '-x', '-1.0', '-y', '0', '-z', '0.01'],
        output='screen')
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='porter_bridge',
        parameters=[{'config_file': str(gazebo_share / 'config/bridge.yaml'),
                     'use_sim_time': True}], output='screen')
    rviz = Node(
        package='rviz2', executable='rviz2', name='porter_rviz',
        arguments=['-d', str(description_share / 'rviz/porter.rviz')],
        parameters=[{'use_sim_time': True}], output='screen')
    behaviours = []
    if behaviour_enabled:
        behaviours.append(Node(
            package='porter_control', executable='color_detector',
            parameters=[str(control_share / 'config/color_detector.yaml')], output='screen'))
        if walker_enabled:
            behaviours.append(Node(
                package='porter_control', executable='person_walker',
                parameters=[str(control_share / 'config/person_walker.yaml')], output='screen'))
        if follow_enabled:
            behaviours.append(Node(
                package='porter_control', executable='follow_controller',
                parameters=[str(control_share / 'config/follow_controller.yaml'),
                            {'use_sim_time': True}], output='screen'))
        else:
            behaviours.append(LogInfo(msg='Automatic following is disabled (follow:=false).'))

    def after_spawn(event, _context):
        if event.returncode != 0:
            return [LogError(msg='Porter could not be spawned; stopping bringup.'),
                    EmitEvent(event=Shutdown(reason='Robot spawn failed'))]
        return [bridge]

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
        RegisterEventHandler(OnProcessExit(
            target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server exited'))])),
        RegisterEventHandler(OnProcessExit(
            target_action=state_publisher,
            on_exit=[EmitEvent(event=Shutdown(reason='Robot state publisher exited'))])),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Sensor bridge exited'))])),
    ]
    after_bridge = [rviz] if rviz_enabled else behaviours
    if after_bridge:
        actions.append(RegisterEventHandler(OnProcessStart(
            target_action=bridge, on_start=after_bridge)))
    if rviz_enabled and behaviours:
        actions.append(RegisterEventHandler(OnProcessStart(
            target_action=rviz, on_start=behaviours)))
    if gui_enabled:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=gui,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo window closed'))])))
    actions.extend([server, state_publisher, spawn])
    if gui_enabled:
        actions.append(gui)
    return actions


def generate_launch_description():
    """Expose the sensor-only view, stationary demo and headless options."""
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Open the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open the prepared three-sensor RViz view'),
        DeclareLaunchArgument('behaviour', default_value='true',
                              description='Run the optional detection and demo nodes'),
        DeclareLaunchArgument('walker', default_value='true',
                              description='Run the 72-second person scenario'),
        DeclareLaunchArgument('follow', default_value='false',
                              description='Enable the controller after implementation is ready'),
        OpaqueFunction(function=_setup),
    ])
