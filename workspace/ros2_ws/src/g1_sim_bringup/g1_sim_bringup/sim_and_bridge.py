#!/usr/bin/env python3

import argparse

from launch import LaunchDescription, LaunchService

from g1_sim_bringup.launch_description import (
    create_runtime_actions,
    _resolve_bridge_config,
    _resolve_robot_model,
    _resolve_world,
)


def main():
    parser = argparse.ArgumentParser(
        description='Launch G1 Gazebo Harmonic simulation and ROS 2 bridge.'
    )
    parser.add_argument(
        '--world',
        default='empty',
        help='Gazebo world alias (empty, g1_free_roam, g1_demo_vel, g1_demo_pos) or absolute path.',
    )
    parser.add_argument(
        '--robot-model',
        default='g1_free_roam',
        help='Robot alias (g1_free_roam, g1_demo_vel, g1_demo_pos, none) or absolute path to model.sdf.',
    )
    parser.add_argument(
        '--bridge-config',
        default='auto',
        help='Bridge alias (clock, g1_vel, g1_pos), YAML path, or auto to infer it from robot_model.',
    )
    parser.add_argument(
        '--gui',
        action=argparse.BooleanOptionalAction,
        default=True,
        help='Launch the Gazebo GUI client.',
    )
    parser.add_argument(
        '--run',
        action=argparse.BooleanOptionalAction,
        default=True,
        help='Start the simulation running immediately.',
    )
    parser.add_argument(
        '--use-software-rendering',
        action=argparse.BooleanOptionalAction,
        default=True,
        help='Force software rendering for the Gazebo GUI inside Docker.',
    )
    parser.add_argument(
        '--verbose',
        default='1',
        help='Gazebo verbosity level.',
    )
    args = parser.parse_args()
    robot_model = _resolve_robot_model(args.robot_model)

    launch_description = LaunchDescription(
        create_runtime_actions(
            world=_resolve_world(args.world),
            robot_model=robot_model,
            bridge_config=_resolve_bridge_config(args.bridge_config, robot_model),
            gui=args.gui,
            run=args.run,
            use_software_rendering=args.use_software_rendering,
            verbose=args.verbose,
        )
    )

    launch_service = LaunchService()
    launch_service.include_launch_description(launch_description)
    return launch_service.run()


if __name__ == '__main__':
    raise SystemExit(main())
