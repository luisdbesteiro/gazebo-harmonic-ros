# Gazebo Harmonic + ROS 2 en Docker

Entorno aislado para ejecutar **Gazebo Harmonic** conectado con **ROS 2 Humble** dentro de Docker, sin modificar la instalacion ROS/Gazebo del host.

El objetivo es reproducir simulaciones del robot Unitree G1 manteniendo separados:

- el host con Ubuntu 22.04, ROS 2 Humble y Gazebo Classic 11
- el contenedor con Gazebo Harmonic y paquetes `ros_gz`
- los modelos, mundos, bridges y resultados curados de este proyecto

## Contenido

```text
.
├── Dockerfile
├── docker-compose.yml
├── README.md
├── SECURITY.md
├── AGENTS.md
├── gazebo_config/
│   └── .gitkeep
└── workspace/
    ├── bridge.yaml
    ├── g1_bridge.yaml
    ├── g1_velocity_bridge.yaml
    ├── G1_LOW_LEVEL.md
    ├── G1_LOW_LEVEL_VELOCITY.md
    ├── models/
    │   ├── g1_description/
    │   ├── g1_29dof_low_level_control/
    │   ├── g1_29dof_low_level_velocity_control/
    │   └── simple_box_robot/
    ├── results/
    ├── ros2_gz_ws/
    │   └── src/
    └── worlds/
```

## Requisitos

- Docker
- Docker Compose
- Servidor X11 disponible si se quiere abrir la GUI de Gazebo desde el contenedor

No hace falta instalar Gazebo Harmonic ni paquetes ROS adicionales en el host.

## Uso

Construir la imagen:

```bash
docker compose build
```

Abrir una shell en el contenedor:

```bash
docker compose run --rm gazebo_harmonic_ros2
```

Ejecutar Gazebo con el mundo de bajo nivel del G1:

```bash
gz sim /workspace/worlds/g1_low_level.world.sdf
```

Ejecutar la variante de control por velocidad articular:

```bash
gz sim /workspace/worlds/g1_low_level_velocity.world.sdf
```

Listar topics de Gazebo:

```bash
gz topic -l
```

Listar topics de ROS 2:

```bash
ros2 topic list
```

Levantar un bridge ROS 2/Gazebo usando la configuracion del workspace:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/workspace/g1_bridge.yaml
```

Para la variante de velocidad articular:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/workspace/g1_velocity_bridge.yaml
```

Si se crean paquetes ROS 2 nuevos, deben vivir en:

```text
workspace/ros2_gz_ws/src/
```

Y compilarse dentro del contenedor:

```bash
cd /workspace/ros2_gz_ws
colcon build --symlink-install
```

## Datos versionados

Se versionan los archivos necesarios para reproducir el entorno:

- `Dockerfile` y `docker-compose.yml`
- documentacion del proyecto
- mundos en `workspace/worlds/`
- modelos y mallas en `workspace/models/`
- configuraciones de bridge en `workspace/*.yaml`
- resultados curados y ligeros en `workspace/results/`

No se versionan cachés, logs ni estado local generado por Gazebo.

## Resultados

Usa `workspace/results/` para publicar resultados pequenos y revisados: resumenes Markdown, tablas CSV, graficas o configuraciones exactas de ejecucion.

Los datos brutos deben ir en `workspace/results/raw/`; esa carpeta esta excluida de Git por defecto para evitar subir bolsas ROS, bases SQLite, videos, logs o datos con rutas locales.

## Seguridad antes de publicar

Este repo ignora por defecto `.env`, claves, tokens, logs, caches de Gazebo/Fuel, builds de colcon y resultados brutos. Antes de hacer `git push`, revisa:

```bash
git ls-files --cached --others --exclude-standard -z | xargs -0 rg -n -i "(api[_-]?key|secret|token|password|passwd|private[_-]?key|authorization|bearer|credential|client[_-]?secret|BEGIN (RSA|OPENSSH|PRIVATE)|ghp_|github_pat_|sk-[A-Za-z0-9])"
git status --ignored --short
```

Mas detalles en [SECURITY.md](SECURITY.md).
