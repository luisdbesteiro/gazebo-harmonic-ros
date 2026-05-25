# G1 Simulation Guide

Esta guia resume el estado actual de las simulaciones del Unitree G1 en este proyecto.

## Variantes disponibles

| Perfil | Mundo | Modelo | Control | Bridge |
| --- | --- | --- | --- | --- |
| Base libre | `/workspace/worlds/g1_free_roam.world.sdf` | `model://g1_29dof` | posicion | `/workspace/bridges/g1_pos_bridge.yaml` |
| Demo fija velocidad | `/workspace/worlds/g1_demo_cmd_vel.world.sdf` | `model://g1_demo_cmd_vel` | velocidad | `/workspace/bridges/g1_vel_bridge.yaml` |
| Demo fija posicion | `/workspace/worlds/g1_demo_cmd_pos.world.sdf` | `model://g1_demo_cmd_pos` | posicion | `/workspace/bridges/g1_pos_bridge.yaml` |

## Arranque manual

Desde el host:

```bash
docker compose up -d gazebo_harmonic_ros2
docker compose exec gazebo_harmonic_ros2 bash
```

Para GPU NVIDIA:

```bash
export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}
docker compose --profile nvidia up -d gazebo_harmonic_ros2_nvidia
docker compose --profile nvidia exec gazebo_harmonic_ros2_nvidia bash
```

Para GPU AMD o Intel por `/dev/dri`:

```bash
export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}
docker compose --profile dri up -d gazebo_harmonic_ros2_dri
docker compose --profile dri exec gazebo_harmonic_ros2_dri bash
```

### Demo fija con control por velocidad

En una shell dentro del contenedor:

```bash
gz sim /workspace/worlds/g1_demo_cmd_vel.world.sdf
```

En otra shell dentro del contenedor:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/workspace/bridges/g1_vel_bridge.yaml
```

Mover una articulacion:

```bash
ros2 topic pub --once /g1/cmd_vel/left_shoulder_pitch_joint std_msgs/msg/Float64 "{data: 0.5}"
```

Pararla:

```bash
ros2 topic pub --once /g1/cmd_vel/left_shoulder_pitch_joint std_msgs/msg/Float64 "{data: 0.0}"
```

Nodo para ejecutar una secuencia de movimientos de ejemplo:
```bash
ros2 run g1_sim_bringup demo_cmd_vel_movements 
```

### Demo fija con control por posicion

En una shell dentro del contenedor:

```bash
gz sim /workspace/worlds/g1_demo_cmd_pos.world.sdf
```

En otra shell dentro del contenedor:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/workspace/bridges/g1_pos_bridge.yaml
```

Mover una articulacion:

```bash
ros2 topic pub --once /g1/cmd_pos/left_shoulder_pitch_joint std_msgs/msg/Float64 "{data: 0.5}"
```

Volver cerca de cero:

```bash
ros2 topic pub --once /g1/cmd_pos/left_shoulder_pitch_joint std_msgs/msg/Float64 "{data: 0.0}"
```

### Base libre

En una shell dentro del contenedor:

```bash
gz sim /workspace/worlds/g1_free_roam.world.sdf
```

En otra shell dentro del contenedor:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/workspace/bridges/g1_pos_bridge.yaml
```

Esta variante deja la base libre en un mundo plano con gravedad y contactos. Es util para pruebas de control y experimentos de red neuronal en condiciones realistas.
Los comandos articulares entran ahora por `/g1/cmd_pos/<joint_name>`.

## Topics expuestos

Todos los perfiles del G1 publican:

```text
/g1/joint_states
/g1/imu/pelvis
/g1/imu/torso
```

En la variante de control por posicion usada para `policy_control` tambien se publica:

```text
/g1/pelvis/odometry
```

El bridge de velocidad configura:

```text
/g1/cmd_vel/<joint_name>
```

El bridge de posicion configura:

```text
/g1/cmd_pos/<joint_name>
```

## Bringup con aliases

Si prefieres `ros2 launch`:

```bash
cd /workspace/ros2_ws
source /workspace/ros2_ws/install/setup.bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py robot_model:=g1_demo_vel spawn_z:=1.2
```

Ejemplo para posicion:

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py robot_model:=g1_demo_pos spawn_z:=1.2
```

Ejemplo para base libre:

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py
```

Si necesitas que la simulacion no empiece automaticamente al lanzar el bringup:

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py robot_model:=g1_free_roam run:=false
```

Con `run:=false`, Gazebo arranca en pausa y puedes iniciar la simulacion manualmente desde la GUI con Play cuando ya hayan arrancado el resto de nodos.

Si necesitas volver temporalmente al renderizado por software:

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py use_software_rendering:=true
```

## Diagnostico rapido

Comprobar topics de Gazebo:

```bash
gz topic -l | grep '^/g1/'
```

Comprobar topics de ROS 2 dentro del contenedor:

```bash
ros2 topic list | grep '^/g1/'
```

Si consultas desde el host:

```bash
export ROS_DOMAIN_ID=42
ros2 topic list | grep '^/g1/'
```
