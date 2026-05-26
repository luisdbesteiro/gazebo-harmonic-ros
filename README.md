# Gazebo Harmonic + ROS 2 en Docker

Entorno aislado para ejecutar Gazebo Harmonic con ROS 2 Humble dentro de Docker, orientado a simulaciones del Unitree G1 sin tocar la instalacion del host.

La documentacion detallada esta repartida por area. Este README deja solo la informacion basica de entrada y enlaza el resto de READMEs del proyecto.

## Documentacion del proyecto

- [workspace/README.md](workspace/README.md): estructura completa de `workspace/`, mundos, modelos, bridges, paquetes ROS 2 y flujo basico de uso.
- [workspace/ros2_ws/src/g1_sim_bringup/README.md](workspace/ros2_ws/src/g1_sim_bringup/README.md): launch principal, aliases de mundos/modelos/bridges, parametros y ejemplos de arranque.
- [workspace/ros2_ws/src/policy_control/README.md](workspace/ros2_ws/src/policy_control/README.md): nodos de observacion e inferencia ONNX, topics, parametros y uso recomendado de la politica.
- [workspace/models/g1_description/README.md](workspace/models/g1_description/README.md): descripcion original URDF/MJCF del Unitree G1 y variantes disponibles.
- [SECURITY.md](SECURITY.md): notas de seguridad para publicar o subir el repositorio.

## Alcance

El repositorio mantiene separados:

- el host con Ubuntu 22.04, ROS 2 Humble y Gazebo Classic 11
- el contenedor con Gazebo Harmonic, ROS 2 Humble y paquetes `ros_gz`
- los recursos propios del proyecto en `workspace/`

No hace falta instalar Gazebo Harmonic ni paquetes ROS adicionales en el host. Si se necesitan dependencias nuevas, deben declararse en el `Dockerfile`.

## Estructura principal

```text
.
├── Dockerfile
├── docker-compose.yml
├── gazebo_config/
└── workspace/
    ├── bridges/
    ├── models/
    ├── results/
    ├── ros2_ws/
    └── worlds/
```

Rutas importantes:

- modelos SDF: `workspace/models/`
- mundos SDF: `workspace/worlds/`
- paquetes ROS 2: `workspace/ros2_ws/src/`
- bridges ROS 2/Gazebo: `workspace/bridges/`

## Requisitos

- Docker
- Docker Compose
- X11 disponible si se quiere abrir la GUI de Gazebo desde el contenedor
- driver NVIDIA y `nvidia-container-toolkit` si se usa el perfil `nvidia`
- acceso a `/dev/dri` si se usa aceleracion AMD/Intel con el perfil `dri`

## Inicio rapido

Construir la imagen:

```bash
docker compose build
```

Levantar el contenedor base:

```bash
docker compose up -d gazebo_harmonic_ros2
```

Entrar al contenedor:

```bash
docker compose exec gazebo_harmonic_ros2 bash
```

Compilar el workspace ROS 2 dentro del contenedor:

```bash
cd /workspace/ros2_ws
colcon build --symlink-install
source /workspace/ros2_ws/install/setup.bash
```

Lanzar la simulacion por defecto:

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py
```

Para arrancar cargado pero en pausa:

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py run:=false
```

## Servicios Docker para ejecución con GPU

El proyecto define tres rutas de arranque:

- `gazebo_harmonic_ros2`: servicio base, sin solicitud explicita de GPU.
- `gazebo_harmonic_ros2_nvidia`: perfil `nvidia`, para hosts con GPU NVIDIA.
- `gazebo_harmonic_ros2_dri`: perfil `dri`, para hosts Linux con AMD/Intel via `/dev/dri`.

Ejemplo con NVIDIA:

```bash
export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}
docker compose --profile nvidia up -d gazebo_harmonic_ros2_nvidia
docker compose --profile nvidia exec gazebo_harmonic_ros2_nvidia bash
```

Ejemplo con AMD/Intel:

```bash
export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}
docker compose --profile dri up -d gazebo_harmonic_ros2_dri
docker compose --profile dri exec gazebo_harmonic_ros2_dri bash
```

Usa solo uno de los servicios a la vez para evitar mezclar sesiones de Gazebo.

## Uso basico

El launch principal esta en `g1_sim_bringup`:

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py world:=empty robot_model:=g1 bridge_config:=auto
```

Aliases principales:

- mundos: `empty`, `rubicon`, `depot`
- modelos: `g1`, `g1_demo_vel`, `g1_demo_pos`, `none`
- bridges: `auto`, `clock`, `g1_vel`, `g1_pos`

Para mas ejemplos y parametros, consulta [g1_sim_bringup/README.md](workspace/ros2_ws/src/g1_sim_bringup/README.md).

## Politica ONNX

El paquete `policy_control` publica observaciones `obs[99]`, ejecuta inferencia ONNX y envia objetivos articulares por posicion.

Flujo minimo, en terminales dentro del contenedor y con la simulacion activa:

```bash
ros2 run policy_control observation_publisher
ros2 run policy_control policy_inference --ros-args -p wrist_scale_factor:=0.0
```

La guia completa de topics, parametros y estado por mundos esta en [policy_control/README.md](workspace/ros2_ws/src/policy_control/README.md).

## Comprobaciones rapidas

Dentro del contenedor:

```bash
gz topic -l
gz service -l
ros2 topic list
```

Si consultas ROS 2 desde el host, usa el mismo dominio DDS que el contenedor:

```bash
export ROS_DOMAIN_ID=42
ros2 topic list
```
