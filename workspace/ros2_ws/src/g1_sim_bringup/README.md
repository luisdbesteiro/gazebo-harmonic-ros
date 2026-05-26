# g1_sim_bringup

Paquete ROS 2 para lanzar Gazebo Harmonic, spawnear el Unitree G1 y arrancar el bridge ROS 2/Gazebo con una unica orden.

## Funcionalidad

El launch principal es:

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py
```

Por defecto:

- abre `/workspace/worlds/empty_world.sdf`
- spawnea `/workspace/models/g1_29dof/model.sdf` con nombre `g1`
- usa `/workspace/bridges/g1_pos_bridge.yaml`
- lanza servidor y GUI de Gazebo
- arranca la simulacion en marcha salvo que se use `run:=false`

## Aliases soportados

### Mundos

| Alias | Ruta | Estado |
| --- | --- | --- |
| `empty` | `/workspace/worlds/empty_world.sdf` | Mundo plano base, recomendado para validar control. |
| `rubicon` | `/workspace/worlds/rubicon.sdf` | Mundo Rubicon de Fuel, con base fisica igualada a `empty_world`. |
| `depot` | `/workspace/worlds/depot.sdf` | Mundo Depot de Fuel, en depuracion para control con politica. |

### Modelos

| Alias | Ruta | Uso |
| --- | --- | --- |
| `g1` | `/workspace/models/g1_29dof/model.sdf` | Robot G1 libre, usado por defecto. |
| `g1_demo_vel` | `/workspace/models/g1_demo_cmd_vel/model.sdf` | Variante fija para demos por velocidad. |
| `g1_demo_pos` | `/workspace/models/g1_demo_cmd_pos/model.sdf` | Variante fija para demos por posicion. |
| `none` | sin modelo | Arranca solo Gazebo y el bridge. |

### Bridges

| Alias | Ruta | Uso |
| --- | --- | --- |
| `auto` | inferido desde `robot_model` | Valor por defecto. |
| `clock` | `/workspace/bridges/clock_bridge.yaml` | Solo `/clock`, usado si `robot_model:=none`. |
| `g1_pos` | `/workspace/bridges/g1_pos_bridge.yaml` | Control por posicion en `/g1/cmd_pos/<joint_name>`. |
| `g1_vel` | `/workspace/bridges/g1_vel_bridge.yaml` | Control por velocidad en `/g1/cmd_vel/<joint_name>`. |

## Argumentos del launch

- `world:=...`: alias de mundo o ruta absoluta a un SDF.
- `robot_model:=...`: alias de modelo, ruta a `model.sdf` o `none`.
- `bridge_config:=auto|...`: bridge inferido automaticamente, alias o ruta YAML.
- `spawn_x`, `spawn_y`, `spawn_z`: posicion inicial del robot.
- `gui:=true|false`: lanza o no el cliente grafico.
- `run:=true|false`: arranca la simulacion en marcha o en pausa.
- `use_software_rendering:=true|false`: fuerza renderizado software para la GUI.
- `verbose:=...`: nivel de verbosidad de Gazebo.

## Spawn por mundo

El mundo `rubicon` tiene una pose por defecto especial para evitar spawnear el G1 dentro del modelo del escenario:

- `spawn_x=-10.0` si no se sobreescribe `spawn_x`
- `spawn_z=4.9` si no se sobreescribe `spawn_z`
- `spawn_y` conserva el valor general, por defecto `0.0`

El resto de mundos usa la pose general por defecto:

- `spawn_x=0.0`
- `spawn_y=0.0`
- `spawn_z=0.80`

## Ejemplos

Mundo vacio en pausa:

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py run:=false
```

Rubicon en pausa, usando la pose especial por defecto:

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py world:=rubicon run:=false
```

Depot en pausa:

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py world:=depot run:=false
```

Arrancar solo Gazebo y `/clock`, sin spawnear robot:

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py robot_model:=none
```

Demo de comandos por velocidad:

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py robot_model:=g1_demo_vel spawn_z:=1.2
ros2 run g1_sim_bringup demo_cmd_vel_movements
```

## Topics principales

Publicados por el robot G1:

```text
/g1/joint_states
/g1/imu/pelvis
/g1/imu/torso
```

En el modelo principal `g1` tambien se expone odometria de pelvis para `policy_control`:

```text
/g1/pelvis/odometry
```

Comandos articulares por posicion:

```text
/g1/cmd_pos/<joint_name>
```

Comandos articulares por velocidad para la demo fija:

```text
/g1/cmd_vel/<joint_name>
```

## Diagnostico rapido

Dentro del contenedor:

```bash
gz topic -l
ros2 topic list
gz service -l
```

Filtrar topics del robot:

```bash
gz topic -l | grep '^/g1/'
ros2 topic list | grep '^/g1/'
```
