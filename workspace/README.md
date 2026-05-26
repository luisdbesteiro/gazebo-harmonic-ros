# Workspace del proyecto

Este directorio agrupa los recursos de simulacion, los paquetes ROS 2 y los resultados asociados al entorno del Unitree G1 en Gazebo Harmonic.

## Estructura de `workspace/`

### `bridges/`

Ficheros YAML para `ros_gz_bridge`. Definen el mapeo entre topics de Gazebo y topics ROS 2.

- `clock_bridge.yaml`: bridge minimo para `/clock`.
- `g1_pos_bridge.yaml`: bridge para control articular por posicion en `/g1/cmd_pos/<joint_name>`.
- `g1_vel_bridge.yaml`: bridge para control articular por velocidad en `/g1/cmd_vel/<joint_name>`.

### `models/`

Modelos SDF y descripciones del robot G1 usados por Gazebo.

- `g1_29dof/`: modelo principal del G1 con 29 grados de libertad y base libre.
- `g1_demo_cmd_pos/`: variante fija levantada sobre el suelo para demos de control por posicion.
- `g1_demo_cmd_vel/`: variante fija levantada sobre el suelo para demos de control por velocidad.
- `g1_description/`: [descripcion oficial de Unitree](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description) utilizada como base: URDF, mallas STL, variantes del robot y recursos asociados.

### `results/`

Resultados del proyecto y material auxiliar de experimentacion.

- contiene capturas, videos y modelos ONNX usados durante las pruebas de control.
- `raw/`: resultados en bruto y artefactos de trabajo, excluído del repositorio git.

### `ros2_ws/`

Workspace ROS 2 dentro del contenedor.

- `src/`: codigo fuente de los paquetes ROS 2 del proyecto.
- `build/`: artefactos generados por `colcon build`.
- `install/`: overlay instalado que debe hacerse `source` antes de lanzar nodos o `ros2 launch`.
- `log/`: logs de compilacion y ejecuciones de `colcon`.

### `worlds/`

Mundos SDF disponibles actualmente para Gazebo Harmonic.

- `empty_world.sdf`: mundo plano base. Es la referencia para validar fisicas, contactos y control.
- `rubicon.sdf`: mundo Rubicon de Fuel. Tiene la misma base de sistemas que `empty_world`; el launch aplica una pose de spawn especial para no colocar el G1 dentro del escenario.
- `depot.sdf`: mundo Depot de Fuel. Esta en depuracion para uso con la politica; se ha ajustado la base fisica, se ha quitado el suelo duplicado y se fuerza el `Depot` a estatico.

## Paquetes ROS 2

### `g1_sim_bringup`

Paquete de arranque de la simulacion. Lanza Gazebo, spawnea el robot y arranca el `parameter_bridge`.

Componentes principales:

- launch `g1_sim_and_bridge.launch.py`: arranca `gz sim`, hace spawn del robot `g1` y levanta el bridge adecuado.
- alias de mundos: `empty`, `rubicon`, `depot`.
- alias de modelos: `g1`, `g1_demo_vel`, `g1_demo_pos`, `none`.
- alias de bridges: `clock`, `g1_vel`, `g1_pos`.
- nodo `demo_cmd_vel_movements`: publica una secuencia ciclica de comandos de velocidad sobre `/g1/cmd_vel/*`.

Parametros utiles del launch:

- `world:=...`: alias o ruta absoluta a un mundo SDF.
- `robot_model:=...`: alias, ruta a `model.sdf` o `none`.
- `bridge_config:=auto|...`: bridge inferido automaticamente, alias o ruta YAML.
- `spawn_x`, `spawn_y`, `spawn_z`: posicion inicial del robot.
- `gui:=true|false`: lanza o no el cliente grafico.
- `run:=true|false`: arranca la simulacion en marcha o en pausa.
- `use_software_rendering:=true|false`: fuerza renderizado software.
- `verbose:=...`: nivel de verbosidad de Gazebo.

Nota sobre `rubicon`: si se usa `world:=rubicon` y no se sobreescriben `spawn_x` ni `spawn_z`, el launch usa `spawn_x:=-10.0` y `spawn_z:=4.9`.

### `policy_control`

Paquete orientado a la integracion de una politica ONNX para el G1 en Gazebo Harmonic.

Nodos incluidos:

- `observation_publisher`: construye y publica la observacion `obs[99]` en `/g1/observation`.
- `policy_inference`: carga la politica ONNX, ejecuta inferencia y publica objetivos articulares en `/g1/cmd_pos/<joint_name>` y la accion previa en `/g1/last_action`.

Entradas principales de `observation_publisher`:

- `/g1/joint_states`
- `/g1/imu/pelvis`
- `/g1/pelvis/odometry`
- `/g1/pelvis_twist` como alternativa
- `/cmd_vel`
- `/clock`

Salidas principales de `policy_control`:

- `/g1/observation`
- `/g1/last_action`
- `/g1/cmd_pos/<joint_name>`

Notas operativas:

- el vector de observacion sigue el orden articular definido en `policy_control/constants.py`
- la ruta ONNX por defecto apunta a `workspace/results/2026-01-26_10-42-02.onnx`
- en las pruebas actuales se recomienda lanzar `policy_inference` con `wrist_scale_factor:=0.0`
- `empty_world.sdf` sigue siendo la referencia mas estable para comparar el comportamiento de la politica

## Uso basico

Los comandos siguientes estan pensados para ejecutarse dentro del contenedor, salvo donde se indique lo contrario.

### 1. Compilar el workspace ROS 2

```bash
cd /workspace/ros2_ws
colcon build --symlink-install
source /workspace/ros2_ws/install/setup.bash
```

### 2. Lanzar simulacion y bridge

Mundo vacio en pausa (recomendado):

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py run:=false
```

### 3. Ejecutar el pipeline de politica

Con la simulacion activa y el bridge de posicion disponible. En otras terminales dentro del docker:

```bash
ros2 run policy_control observation_publisher
ros2 run policy_control policy_inference --ros-args -p wrist_scale_factor:=0.0
```

### 4. Comprobaciones rapidas

```bash
gz topic -l
ros2 topic list
gz service -l
```

## Documentacion relacionada

- `ros2_ws/src/g1_sim_bringup/README.md`: uso detallado del launch y aliases actuales.
- `G1_SIM_GUIDE.md`: guia operativa historica de simulacion.
- `models/g1_description/README.md`: informacion especifica sobre la descripcion del robot.
