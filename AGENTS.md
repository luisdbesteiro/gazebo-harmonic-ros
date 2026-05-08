# Proyecto Gazebo Harmonic + ROS 2 en Docker

Este proyecto debe mantenerse aislado dentro de:

`~/docker/gazebo-harmonic-ros/`

## Reglas obligatorias

- No modificar archivos fuera de este directorio.
- No instalar paquetes en el host con `apt`, `pip`, `conda`, `snap` o similares.
- No ejecutar `sudo`.
- No modificar `~/.bashrc`, `~/.profile`, `~/.zshrc`, `/etc/*`, `/opt/ros/*` ni `~/ros2_ws`.
- No tocar la instalación del host de ROS 2 Humble ni Gazebo Classic 11.
- Todo lo relacionado con Gazebo Harmonic debe ir dentro del Docker.
- Los modelos deben ir en `workspace/models/`.
- Los mundos deben ir en `workspace/worlds/`.
- Los paquetes ROS 2 nuevos deben ir en `workspace/ros2_gz_ws/src/`.
- Si hace falta instalar dependencias, deben añadirse al `Dockerfile`, no instalarse manualmente en el host.
- Antes de proponer cambios destructivos, explicar qué se va a modificar.

## Comandos permitidos habituales

- `docker compose build`
- `docker compose run --rm gazebo_harmonic_ros2`
- `docker compose exec gazebo_harmonic_ros2 bash`
- `colcon build --symlink-install`
- `gz sim`
- `gz topic -l`
- `gz service -l`
- `ros2 topic list`
- `ros2 run ros_gz_bridge parameter_bridge`

## Objetivo

Crear un entorno aislado para simulaciones del robot unitree G1 con Gazebo Harmonic conectado con ROS 2, sin romper la configuración actual del host.
