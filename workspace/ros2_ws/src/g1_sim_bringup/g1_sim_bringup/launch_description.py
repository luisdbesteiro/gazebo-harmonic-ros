import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration


WORLD_ALIASES = {
    'empty': '/workspace/worlds/empty_world.sdf',
    'g1_free_roam': '/workspace/worlds/g1_free_roam.world.sdf',
    'g1_demo_vel': '/workspace/worlds/g1_demo_cmd_vel.world.sdf',
    'g1_demo_pos': '/workspace/worlds/g1_demo_cmd_pos.world.sdf',
}

ROBOT_ALIASES = {
    'g1_free_roam': '/workspace/models/g1_29dof/model.sdf',
    'g1_demo_vel': '/workspace/models/g1_demo_cmd_vel/model.sdf',
    'g1_demo_pos': '/workspace/models/g1_demo_cmd_pos/model.sdf',
}

BRIDGE_ALIASES = {
    'clock': '/workspace/bridges/clock_bridge.yaml',
    'g1_vel': '/workspace/bridges/g1_vel_bridge.yaml',
    'g1_pos': '/workspace/bridges/g1_pos_bridge.yaml',
}


def _as_bool(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _resolve_world(world):
    return WORLD_ALIASES.get(world, world)


def _resolve_robot_model(robot_model):
    if robot_model == 'none':
        return None

    resolved = ROBOT_ALIASES.get(robot_model, robot_model)
    if os.path.isdir(resolved):
        return os.path.join(resolved, 'model.sdf')
    return resolved


def _resolve_bridge_config(bridge_config, robot_model):
    if bridge_config != 'auto':
        return BRIDGE_ALIASES.get(bridge_config, bridge_config)

    if robot_model is None:
        return BRIDGE_ALIASES['clock']

    if robot_model == ROBOT_ALIASES['g1_demo_pos']:
        return BRIDGE_ALIASES['g1_pos']

    if robot_model in (
        ROBOT_ALIASES['g1_demo_vel'],
        ROBOT_ALIASES['g1_free_roam'],
    ):
        return BRIDGE_ALIASES['g1_vel']

    raise ValueError(
        'bridge_config:=auto only supports robot_model:=g1_free_roam, robot_model:=g1_demo_vel or robot_model:=g1_demo_pos. Set bridge_config explicitly for custom models.'
    )


def create_runtime_actions(
    *,
    world,
    robot_model,
    bridge_config,
    gui,
    run,
    use_software_rendering,
    verbose,
):
    gazebo_env = {
        'GZ_SIM_RESOURCE_PATH': '/workspace/models:/workspace/worlds',
    }

    if use_software_rendering:
        gazebo_env.update(
            {
                'LIBGL_ALWAYS_SOFTWARE': '1',
                'MESA_LOADER_DRIVER_OVERRIDE': 'llvmpipe',
            }
        )

    server_cmd = ['gz', 'sim', '-s']
    if run:
        server_cmd.append('-r')
    server_cmd.extend(['-v', str(verbose), world])

    actions = [
        ExecuteProcess(
            cmd=server_cmd,
            output='screen',
            additional_env=gazebo_env,
        ),
    ]

    if robot_model is not None:
        actions.append(
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'ros2',
                            'run',
                            'ros_gz_sim',
                            'create',
                            '-file',
                            robot_model,
                            '-name',
                            'g1',
                            '-x',
                            '0',
                            '-y',
                            '0',
                            '-z',
                            '1.20',
                        ],
                        output='screen',
                    )
                ],
            )
        )

    actions.append(
        ExecuteProcess(
            cmd=[
                'ros2',
                'run',
                'ros_gz_bridge',
                'parameter_bridge',
                '--ros-args',
                '-p',
                f'config_file:={bridge_config}',
            ],
            output='screen',
        )
    )

    if gui:
        actions.append(
            ExecuteProcess(
                cmd=['gz', 'sim', '-g', '-v', str(verbose)],
                output='screen',
                additional_env=gazebo_env,
            )
        )

    return actions


def _create_runtime_actions_from_launch_args(context):
    robot_model = _resolve_robot_model(
        LaunchConfiguration('robot_model').perform(context)
    )
    return create_runtime_actions(
        world=_resolve_world(LaunchConfiguration('world').perform(context)),
        robot_model=robot_model,
        bridge_config=_resolve_bridge_config(
            LaunchConfiguration('bridge_config').perform(context),
            robot_model,
        ),
        gui=_as_bool(LaunchConfiguration('gui').perform(context)),
        run=_as_bool(LaunchConfiguration('run').perform(context)),
        use_software_rendering=_as_bool(
            LaunchConfiguration('use_software_rendering').perform(context)
        ),
        verbose=LaunchConfiguration('verbose').perform(context),
    )


def create_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value='empty',
                description='Gazebo world alias (empty, g1_free_roam, g1_demo_vel, g1_demo_pos) or absolute path.',
            ),
            DeclareLaunchArgument(
                'robot_model',
                default_value='g1_free_roam',
                description='Robot alias (g1_free_roam, g1_demo_vel, g1_demo_pos, none) or absolute path to model.sdf.',
            ),
            DeclareLaunchArgument(
                'bridge_config',
                default_value='auto',
                description='Bridge YAML path, or auto to infer it from robot_model.',
            ),
            DeclareLaunchArgument(
                'gui',
                default_value='true',
                description='Launch the Gazebo GUI client.',
            ),
            DeclareLaunchArgument(
                'run',
                default_value='true',
                description='Start the simulation running immediately.',
            ),
            DeclareLaunchArgument(
                'use_software_rendering',
                default_value='true',
                description='Force software rendering for the Gazebo GUI inside Docker.',
            ),
            DeclareLaunchArgument(
                'verbose',
                default_value='1',
                description='Gazebo verbosity level.',
            ),
            OpaqueFunction(function=_create_runtime_actions_from_launch_args),
        ]
    )
