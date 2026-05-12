# Gazebo Harmonic + ROS 2 en Docker

Entorno aislado para ejecutar Gazebo Harmonic con ROS 2 Humble dentro de Docker, sin tocar la instalacion del host.

El repositorio mantiene separados:

- el host con Ubuntu 22.04, ROS 2 Humble y Gazebo Classic 11
- el contenedor con Gazebo Harmonic y paquetes `ros_gz`
- los modelos, mundos, bridges y bringup ROS 2 propios de este proyecto

## Estado actual

El workspace contiene tres variantes principales del Unitree G1:

- `g1_29dof`: base libre en microgravedad, control por velocidad
- `g1_demo_cmd_vel`: pelvis fija al mundo, control por velocidad
- `g1_demo_cmd_pos`: pelvis fija al mundo, control por posicion

Y tres mundos principales:

- `workspace/worlds/g1_free_roam.world.sdf`
- `workspace/worlds/g1_demo_cmd_vel.world.sdf`
- `workspace/worlds/g1_demo_cmd_pos.world.sdf`

Los bridges disponibles son:

- `workspace/bridges/clock_bridge.yaml`
- `workspace/bridges/g1_vel_bridge.yaml`
- `workspace/bridges/g1_pos_bridge.yaml`

## Estructura

```text
.
├── Dockerfile
├── docker-compose.yml
├── README.md
├── SECURITY.md
├── AGENTS.md
├── gazebo_config/
└── workspace/
    ├── G1_SIM_GUIDE.md
    ├── bridges/
    ├── models/
    │   ├── g1_29dof/
    │   ├── g1_demo_cmd_pos/
    │   ├── g1_demo_cmd_vel/
    │   └── g1_description/
    ├── results/
    ├── ros2_ws/
    │   └── src/g1_sim_bringup/
    └── worlds/
```

## Requisitos

- Docker
- Docker Compose
- X11 disponible si quieres abrir la GUI de Gazebo desde el contenedor

No hace falta instalar Gazebo Harmonic ni paquetes ROS adicionales en el host.

## Inicio rapido

Construir la imagen:

```bash
docker compose build
```

Levantar el contenedor:

```bash
docker compose up -d gazebo_harmonic_ros2
```

Abrir una shell:

```bash
docker compose exec gazebo_harmonic_ros2 bash
```

Dentro del contenedor puedes lanzar, por ejemplo, la variante fija con control por velocidad:

```bash
gz sim /workspace/worlds/g1_demo_cmd_vel.world.sdf
```

Y en otra shell del mismo contenedor levantar el bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/workspace/bridges/g1_vel_bridge.yaml
```

Si consultas ROS 2 desde el host, usa el mismo dominio DDS:

```bash
export ROS_DOMAIN_ID=42
ros2 topic list
```

## Bringup ROS 2

El paquete `workspace/ros2_ws/src/g1_sim_bringup` ofrece un launch y un ejecutable para arrancar simulacion y bridge con aliases coherentes.

Ejemplo con `ros2 launch`:

```bash
cd /workspace/ros2_ws
colcon build --symlink-install
source /workspace/ros2_ws/install/setup.bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py world:=g1_demo_vel robot_model:=g1_demo_vel bridge_config:=auto
```

Aliases soportados actualmente:

- `world:=empty|g1_free_roam|g1_demo_vel|g1_demo_pos`
- `robot_model:=none|g1_free_roam|g1_demo_vel|g1_demo_pos`
- `bridge_config:=auto|clock|g1_vel|g1_pos|/ruta/a/archivo.yaml`

## Modelos y mundos

Consulta [workspace/G1_SIM_GUIDE.md](/home/luis/jderobot/gazebo-harmonic-ros/workspace/G1_SIM_GUIDE.md:1) para ver la matriz completa modelo-mundo-bridge y ejemplos de control por velocidad y posicion.

La descripcion URDF/MJCF original de Unitree permanece en `workspace/models/g1_description/`.

## Desarrollo

Si se crean paquetes ROS 2 nuevos, deben vivir en:

```text
workspace/ros2_ws/src/
```

Y compilarse dentro del contenedor:

```bash
cd /workspace/ros2_ws
colcon build --symlink-install
```


Detalles de seguridad en la subida en [SECURITY.md](SECURITY.md).
