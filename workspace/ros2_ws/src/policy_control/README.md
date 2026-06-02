# policy_control

Paquete ROS 2 para ejecutar una politica ONNX de locomocion del Unitree G1 en Gazebo Harmonic.

El paquete conecta el estado simulado del robot con el formato de observacion usado por la politica entrenada en MuJoCo, ejecuta inferencia y publica objetivos articulares por posicion.

Para depuracion con nodos ROS se recomienda usar el nodo C++ `policy_controller_cpp`, porque reduce procesos, copias ROS intermedias y coste de memoria frente al par de nodos Python. El launch por defecto de `g1_sim_bringup` usa `g1_policy_plugin`, que ejecuta la politica dentro de Gazebo. Los nodos Python se mantienen como recurso de depuracion y por claridad, ya que separan explicitamente la construccion de la observacion y la inferencia.

## Estructura del paquete

`policy_control` es un paquete mixto `ament_cmake` + `ament_cmake_python`. La separacion de carpetas sigue la convencion habitual de ROS 2 para combinar nodos C++, plugins y modulos Python en un mismo paquete:

- `src/`: fuentes C++. Contiene el nodo integrado `policy_controller_cpp` y el plugin de sistema `policy_gz_system`.
- `include/policy_control/`: cabeceras C++ compartidas por los ejecutables y plugins del paquete.
- `policy_control/`: modulo Python importable. Contiene los nodos Python y utilidades comunes, como constantes de articulaciones y escalas.
- `scripts/`: wrappers ejecutables Python instalados en `lib/policy_control` para poder lanzar los nodos con `ros2 run`.
- `env-hooks/`: hooks de entorno instalados por `ament` para exponer rutas necesarias en ejecucion, como la ruta del plugin de Gazebo.
- `resource/`: marcador usado por el indice de paquetes de `ament`.

Los nodos quedan disponibles de forma uniforme tras compilar el workspace:

```bash
ros2 run policy_control policy_controller_cpp
ros2 run policy_control observation_publisher
ros2 run policy_control policy_inference
```

## Nodos

### `observation_publisher`

Construye y publica el vector `obs[99]` en `/g1/observation`.

Entradas por defecto:

- `/g1/joint_states`
- `/g1/imu/pelvis`
- `/g1/pelvis/odometry`
- `/g1/pelvis_twist`
- `/cmd_vel`
- `/g1/last_action`
- `/clock`

Salida por defecto:

- `/g1/observation`

El nodo publica al ritmo de tiempo de simulacion, no con reloj de pared. Solo genera una observacion nueva cuando avanza `/clock`, respetando `publish_rate_hz`, que por defecto es `50.0`.

### `policy_inference`

Carga la politica ONNX, consume `/g1/observation` y publica:

- `/g1/last_action`
- `/g1/cmd_pos/<joint_name>`

La inferencia se ejecuta al recibir cada observacion valida. No usa timers de reloj de pared, asi que queda acoplada al avance real de la simulacion.

### `policy_controller_cpp`

Nodo C++ integrado que sustituye al par `observation_publisher` + `policy_inference` y ejecuta todo el lazo en un solo proceso.

Entradas por defecto:

- `/g1/joint_states`
- `/g1/imu/pelvis`
- `/g1/pelvis/odometry`
- `/g1/pelvis_twist`
- `/cmd_vel`
- `/clock`

Salidas por defecto:

- `/g1/last_action`
- `/g1/cmd_pos/<joint_name>`

El nodo construye internamente `obs[99]`, ejecuta la politica ONNX con ONNX Runtime C++ y publica los objetivos articulares. Por defecto no publica `/g1/observation`, se puede activar con `publish_observation:=true`.

### `policy_gz_system`

Plugin de sistema para Gazebo Harmonic. Lee el estado interno del modelo desde Gazebo, consume solo `/cmd_vel` desde ROS 2, ejecuta ONNX Runtime C++ dentro del servidor de simulacion y publica los objetivos a los topics Gazebo `/g1/cmd_pos/<joint_name>`.

Esta primera version mantiene la salida por Gazebo Transport para reutilizar los `JointPositionController` ya definidos en el SDF del G1. No sustituye todavia los controladores articulares de Gazebo.

## Estructura de `obs[99]`

La observacion mantiene el orden esperado por la politica:

1. velocidad lineal de pelvis: 3
2. velocidad angular de pelvis: 3
3. gravedad proyectada en pelvis: 3
4. posiciones articulares relativas a la pose nominal: 29
5. velocidades articulares: 29
6. `last_action`: 29
7. comando `[vx, vy, wz]`: 3

Total: `99` componentes.

El orden de articulaciones, la pose nominal y las escalas de accion estan definidos en:

- `policy_control/constants.py`

## Semantica del control

La salida de la red se interpreta igual que en la referencia de MuJoCo:

```text
raw_action = policy(obs)
last_action = raw_action
joint_target = default_joint_pos + scaled_action * action_scale * action_scale_factor
```

`last_action` guarda la salida cruda de la politica. Los objetivos articulares publicados en `/g1/cmd_pos/<joint_name>` aplican los factores de escala configurados.

## Parametros principales

### `observation_publisher`

| Parametro | Valor por defecto |
| --- | --- |
| `joint_state_topic` | `/g1/joint_states` |
| `pelvis_imu_topic` | `/g1/imu/pelvis` |
| `pelvis_odometry_topic` | `/g1/pelvis/odometry` |
| `pelvis_twist_topic` | `/g1/pelvis_twist` |
| `command_topic` | `/cmd_vel` |
| `last_action_topic` | `/g1/last_action` |
| `observation_topic` | `/g1/observation` |
| `clock_topic` | `/clock` |
| `publish_rate_hz` | `50.0` |

### `policy_inference`

| Parametro | Valor por defecto |
| --- | --- |
| `observation_topic` | `/g1/observation` |
| `last_action_topic` | `/g1/last_action` |
| `cmd_pos_prefix` | `/g1/cmd_pos` |
| `onnx_model_path` | `workspace/results/2026-01-26_10-42-02.onnx` si existe |
| `clock_topic` | `/clock` |
| `clip_action` | `false` |
| `action_clip_min` | `-100.0` |
| `action_clip_max` | `100.0` |
| `action_scale_factor` | `1.0` |
| `upper_body_scale_factor` | `1.0` |
| `wrist_scale_factor` | `1.0` |
| `hold_nominal_pose_duration_s` | `0.0` |

El nodo requiere `onnxruntime` en el entorno Python del contenedor. Si no esta disponible, falla al arrancar con un error explicito.

### `policy_controller_cpp`

Usa los mismos parametros de entrada y control que `observation_publisher` y `policy_inference`, mas:

| Parametro | Valor por defecto |
| --- | --- |
| `publish_observation` | `false` |

El nodo requiere ONNX Runtime C/C++ en la imagen Docker. El `Dockerfile` lo instala en `/opt/onnxruntime`; despues de cambiar la imagen hay que reconstruirla.

### `policy_gz_system`

Se configura desde el bloque SDF del plugin:

| Parametro SDF | Valor por defecto |
| --- | --- |
| `onnx_model_path` | `/workspace/results/2026-01-26_10-42-02.onnx` |
| `cmd_vel_topic` | `/cmd_vel` |
| `cmd_pos_prefix` | `/g1/cmd_pos` |
| `pelvis_link_name` | `pelvis` |
| `control_rate_hz` | `50.0` |
| `publish_metrics` | `false` |
| `metrics_topic` | `/g1/policy_metrics` |
| `clip_action` | `false` |
| `action_clip_min` | `-100.0` |
| `action_clip_max` | `100.0` |
| `action_scale_factor` | `1.0` |
| `upper_body_scale_factor` | `1.0` |
| `wrist_scale_factor` | `1.0` |
| `hold_nominal_pose_duration_s` | `0.0` |

Si `publish_metrics` esta activo, `/g1/policy_metrics` publica un `Float64MultiArray` con tiempos en milisegundos: construccion de observacion, inferencia, publicacion y total.

## Uso recomendado

Hay dos rutas de uso:

- `g1_low_level`: depuracion y pruebas de la politica con nodos ROS 2. Expone `/g1/cmd_pos/<joint_name>` a ROS mediante `ros_gz_bridge`.
- `g1_policy_plugin`: ruta final prevista. Gazebo carga `policy_gz_system`, consume `/cmd_vel` desde ROS 2 y publica internamente los targets articulares por Gazebo Transport.

Para depurar con nodos ROS, primero lanza la simulacion con `g1_low_level` y bridge de posicion.

Mundo vacio, referencia mas estable para comparar la politica:

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py robot_model:=g1_low_level run:=false
```

Despues, en otras terminales dentro del contenedor:

```bash
ros2 run policy_control policy_controller_cpp --ros-args -p wrist_scale_factor:=0.0
```
La recomendacion actual para Gazebo es usar `wrist_scale_factor:=0.0`, salvo que se este depurando especificamente el efecto de las muñecas.

Ruta Python para depuracion:

```bash
ros2 run policy_control observation_publisher
ros2 run policy_control policy_inference --ros-args -p wrist_scale_factor:=0.0
```

Si se quiere inspeccionar tambien la observacion generada por el nodo C++:

```bash
ros2 run policy_control policy_controller_cpp --ros-args -p wrist_scale_factor:=0.0 -p publish_observation:=true
```

Con todo esto cargado, el robot está listo para recibir comandos de velocidad y dirección deseadas desde el topic `/cmd_vel`. Una buena forma de enviarlos es usando `teleop_twist_keyboard` desde otra terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Plugin Gazebo

El plugin se carga en la variante `g1_policy_plugin`, que tambien es el modelo por defecto de `g1_sim_bringup`. No hay que lanzar a la vez `policy_controller_cpp`, porque ambos escribirian en `/g1/cmd_pos/<joint_name>` desde caminos distintos.

La variante vive en:

```text
/workspace/models/g1_policy_plugin/model.sdf
```

Bloque SDF usado dentro de `<model>`:

```xml
<plugin filename="libpolicy_gz_system.so" name="policy_control::PolicyGzSystem">
  <onnx_model_path>/workspace/results/2026-01-26_10-42-02.onnx</onnx_model_path>
  <cmd_vel_topic>/cmd_vel</cmd_vel_topic>
  <cmd_pos_prefix>/g1/cmd_pos</cmd_pos_prefix>
  <control_rate_hz>50.0</control_rate_hz>
  <wrist_scale_factor>0.0</wrist_scale_factor>
  <publish_metrics>false</publish_metrics>
</plugin>
```

Al arrancar Gazebo desde un entorno que haya hecho `source /workspace/ros2_ws/install/setup.bash`, la libreria queda disponible en el install del workspace. Esta version sigue usando los `gz-sim-joint-position-controller-system` existentes para aplicar cada target articular.

Lanzamiento recomendado para esta variante:

```bash
ros2 launch g1_sim_bringup g1_sim_and_bridge.launch.py robot_model:=g1_policy_plugin run:=false
```

## Ejemplos de depuracion

Atenuar toda la accion:

```bash
ros2 run policy_control policy_inference --ros-args -p action_scale_factor:=0.5
```

Mantener la pose nominal al inicio:

```bash
ros2 run policy_control policy_inference --ros-args -p hold_nominal_pose_duration_s:=1.0
```

Atenuar solo tren superior:

```bash
ros2 run policy_control policy_inference --ros-args -p upper_body_scale_factor:=0.2
```

Fijar las muñecas en la pose nominal desde el punto de vista del comando:

```bash
ros2 run policy_control policy_inference --ros-args -p wrist_scale_factor:=0.0
```

Sobreescribir la ruta del modelo ONNX:

```bash
ros2 run policy_control policy_inference --ros-args -p onnx_model_path:=/workspace/results/2026-01-26_10-42-02.onnx
```

## Comprobaciones rapidas

Con `g1_low_level` y nodos ROS, verificar que llegan entradas:

```bash
ros2 topic hz /g1/joint_states
ros2 topic hz /g1/imu/pelvis
ros2 topic hz /g1/pelvis/odometry
```

Verificar el lazo de politica con nodos ROS:

```bash
ros2 topic hz /g1/observation
ros2 topic hz /g1/last_action
ros2 topic list | grep '^/g1/cmd_pos/'
```

Con `g1_policy_plugin`, los targets articulares no pasan por ROS; se comprueban en Gazebo Transport:

```bash
gz topic -l | grep '^/g1/cmd_pos/'
```

Enviar comandos externos para la politica:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Estado por mundos

| Mundo | Estado para politica |
| --- | --- |
| `empty` | Referencia principal. Es el entorno mas estable para comparar cambios de control. |
| `rubicon` | Compatible con spawn especial desde `g1_sim_bringup`; requiere validacion de la politica sobre geometria no plana. |
| `depot` | En depuracion. Se han ajustado fisicas, suelo duplicado y estaticidad del modelo, pero puede seguir comportandose peor que `empty`. |

## Pasos futuros

- La velocidad en la pelvis no es un sensor nativo equivalente al velocimetro de MuJoCo, la IMU de Gazebo aporta orientacion, velocidad angular y aceleracion lineal, pero no velocidad lineal. Esto puede afrontarse haciendo un plugin de sensor personalizado en Gazebo.
- Todavía no hay launch propio de `policy_control`; se lanza junto a una simulacion ya arrancada con `g1_sim_bringup`.
- Descartar los nodos Python y la dependencia `onnxruntime` del entorno Python del contenedor.
- Comparar `policy_controller_cpp` con `policy_gz_system` en estabilidad, latencia y coste antes de convertir el plugin en la ruta recomendada.
- Explorar una segunda fase del plugin que escriba comandos articulares directamente en componentes internos de Gazebo, sin publicar targets por Gazebo Transport.
