# policy_control

Paquete ROS 2 para preparar la integracion de la politica neuronal del G1 en Gazebo Harmonic.

## Estado actual

El paquete esta en una fase inicial. Ahora mismo contiene dos nodos:

- `observation_publisher`
- `policy_inference`

Su objetivo es reconstruir y publicar el vector de observacion `obs[99]` que espera la politica entrenada en MuJoCo.

## Lo que hace ahora mismo

El nodo `observation_publisher`:

- subscribe a `/g1/joint_states`
- subscribe a `/g1/imu/pelvis`
- subscribe a `/g1/pelvis/odometry`
- subscribe a `/clock`
- subscribe a `/cmd_vel`
- subscribe a `/g1/last_action`
- subscribe opcionalmente a `/g1/pelvis_twist`
- publica la observacion completa en `/g1/observation` como `std_msgs/msg/Float64MultiArray`

El nodo `policy_inference`:

- subscribe a `/g1/observation`
- subscribe a `/clock`
- carga la politica ONNX
- publica `last_action` en `/g1/last_action`
- publica objetivos articulares en `/g1/cmd_pos/<joint_name>`

El lazo ya no depende de timers de reloj de pared:

- `observation_publisher` publica solo cuando avanza `/clock`
- `policy_inference` hace inferencia al recibir cada observacion nueva

Esto evita que la politica siga iterando mientras Gazebo esta en pausa con `run:=false` y se ajusta a los cambios reales en la simulacion.

La semantica del control replica la referencia MuJoCo:

- `raw_action = policy(obs)`
- `last_action = raw_action`
- `joint_target = default_joint_pos + raw_action * action_scale`

Opcionalmente, `policy_inference` permite varios mecanismos de depuracion:

- `action_scale_factor`: factor global multiplicativo sobre `raw_action * action_scale`
- `upper_body_scale_factor`: factor multiplicativo aplicado solo a hombros, codos y munecas
- `wrist_scale_factor`: factor multiplicativo aplicado solo a `wrist_roll`, `wrist_pitch` y `wrist_yaw`
- `hold_nominal_pose_duration_s`: tiempo inicial en segundos de simulacion durante el que se publica solo `DEFAULT_JOINT_POS` y `last_action = 0`

Estado actual para Gazebo:

- `wrist_scale_factor` sigue teniendo valor por defecto `1.0`
- aun asi, la configuracion que mejor estabilidad ha dado en simulacion es lanzar `policy_inference` con `wrist_scale_factor:=0.0`
- esto deja las munecas en pose nominal desde el punto de vista de la politica y evita perturbaciones que no estaban ayudando al equilibrio

El orden de articulaciones y la pose nominal de referencia estan codificados en:

- `policy_control/constants.py`

La pose nominal usada es la `KNEES_BENT_KEYFRAME` de la referencia MuJoCo.

## Estructura de la observacion

La observacion publicada sigue esta estructura:

1. velocidad lineal de pelvis: 3
2. velocidad angular de pelvis: 3
3. gravedad proyectada en pelvis: 3
4. posiciones articulares relativas a la pose nominal: 29
5. velocidades articulares: 29
6. `last_action`: 29
7. comando `[vx, vy, wz]`: 3

Total:

- `99` componentes

## Limitaciones actuales

Este paquete todavia no reproduce completamente la observacion de MuJoCo.

Limitaciones conocidas:

- la nueva fuente principal de velocidad lineal es `/g1/pelvis/odometry`, publicada por `OdometryPublisher`
- esa velocidad lineal no sale del sensor IMU de Gazebo, sino de un publisher de odometria basado en el estado cinemático del modelo
- la IMU estandar de Gazebo publica orientacion, velocidad angular y aceleracion lineal, pero no velocidad lineal
- esto aproxima el `velocimeter` de MuJoCo, pero no es un sensor equivalente ni una IMU realista
- si no llega ni `/g1/pelvis/odometry` ni `/g1/pelvis_twist`, las primeras 3 componentes de la observacion se publican a cero
- no hay launch propio del paquete
- no se ha validado aun la observacion contra trazas de MuJoCo
- falta validar en simulacion que las ganancias PD de Gazebo permiten que la politica se comporte de forma parecida a MuJoCo

## Nota sobre sensores en Gazebo

En Gazebo Harmonic no estamos usando un sensor nativo tipo `velocimeter` como el de MuJoCo.

Segun la documentacion oficial:

- la IMU de Gazebo publica orientacion, velocidad angular y aceleracion lineal
- la lista oficial de sensores soportados no incluye un sensor generico de velocidad lineal para un link terrestre equivalente al `velocimeter` de MuJoCo

Referencias:

- sensores de Gazebo: https://gazebosim.org/libs/sensors/
- tutorial de IMU en Gazebo Harmonic: https://gazebosim.org/docs/harmonic/sensors/
- API de `OdometryPublisher`: https://gazebosim.org/api/sim/9/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html

Por eso, de momento, la velocidad lineal de pelvis se obtiene desde `OdometryPublisher` como una aproximacion funcional para reproducir la entrada que esperaba la politica.

Si mas adelante hace falta una solucion mas fiel, hay dos caminos razonables:

- mantener `OdometryPublisher` solo para validacion inicial de la politica
- implementar un plugin o sensor custom que publique la velocidad lineal de pelvis en el frame del IMU, con el ruido y la dinamica que interese simular

## Topics usados por defecto

Entradas:

- `/g1/joint_states`
- `/g1/imu/pelvis`
- `/g1/pelvis/odometry`
- `/g1/pelvis_twist`
- `/cmd_vel`
- `/g1/last_action`
- `/g1/observation`
- `/clock`

Salida:

- `/g1/observation`
- `/g1/last_action`
- `/g1/cmd_pos/<joint_name>`

Todos son parametrizables desde ROS.

En particular, `/cmd_vel` se interpreta como:

- `linear.x -> vx`
- `linear.y -> vy`
- `angular.z -> wz`

`/g1/last_action` contiene la salida ONNX cruda en el mismo orden articular de la politica.

## Politica ONNX

Por defecto, `policy_inference` intenta cargar:

- `workspace/results/raw/2025-tfg-diego-lopez/pruebas/G1/2026-01-26_10-42-02.onnx`

La ruta puede sobreescribirse con el parametro ROS `onnx_model_path`.

El nodo requiere `onnxruntime` en el entorno Python del contenedor. Si no esta disponible, el nodo fallara al arrancar con un error explicito.

Parametros utiles de `policy_inference`:

- `action_scale_factor:=1.0`
- `upper_body_scale_factor:=1.0`
- `wrist_scale_factor:=1.0`
- `hold_nominal_pose_duration_s:=0.0`

Recomendacion actual para Gazebo:

- lanzar `policy_inference` con `wrist_scale_factor:=0.0` salvo que se este depurando especificamente el efecto de las munecas

Ejemplo conservador para depuracion:

```bash
ros2 run policy_control policy_inference --ros-args -p action_scale_factor:=0.5 -p hold_nominal_pose_duration_s:=1.0
```

Ejemplo para atenuar movimientos del tren superior:

```bash
ros2 run policy_control policy_inference --ros-args -p upper_body_scale_factor:=0.2
```

Ejemplo para aislar especificamente el problema de las munecas:

```bash
ros2 run policy_control policy_inference --ros-args -p wrist_scale_factor:=0.0
```

Ejemplo recomendado de lanzamiento en Gazebo:

```bash
ros2 run policy_control policy_inference --ros-args -p wrist_scale_factor:=0.0
```

## Estado de validacion

Validado hasta ahora:

- coherencia interna del vector `obs[99]`
- uso del mismo orden articular que la politica de MuJoCo
- build del paquete `policy_control`
- lazo de observacion e inferencia sincronizado con tiempo de simulacion
- mejora clara de estabilidad en Gazebo al lanzar `policy_inference` con `wrist_scale_factor:=0.0`

Frecuencias esperadas en tiempo de simulacion:

| Componente | Frecuencia teorica |
| --- | ---: |
| fisica Gazebo (`max_step_size = 1 ms`) | 1000 Hz |
| IMU pelvis | 400 Hz |
| odometria pelvis | 400 Hz |
| `observation_publisher` | 50 Hz |
| `policy_inference` | 50 Hz |

Frecuencias medidas en una ejecucion real dentro del contenedor, contra reloj de pared:

| Topic / componente | Frecuencia observada |
| --- | ---: |
| `/g1/joint_states` | ~654-694 Hz |
| `/g1/imu/pelvis` | ~336-355 Hz |
| `/g1/pelvis/odometry` | ~218-229 Hz |
| `/g1/observation` | ~33-35 Hz |
| `/g1/last_action` | ~32-34 Hz |
| `/clock` | ~590-660 Hz |

Conclusiones cortas:

- `observation_publisher` y `policy_inference` quedan acoplados entre si y avanzan juntos
- la observacion y `last_action` se mueven en el mismo orden de frecuencia, como se espera
- la diferencia entre los 50 Hz teoricos y los ~33-35 Hz medidos se explica porque la simulacion no iba a `real_time_factor = 1` de forma sostenida
- durante la medicion, Gazebo mantuvo `step_size = 1 ms`, asi que el ritmo correcto del lazo sigue estando definido por tiempo de simulacion y no por reloj de pared

Pendiente:

- validacion funcional completa del control en la simulacion
- validacion numerica contra MuJoCo
- validacion de `/g1/pelvis/odometry` como sustituto del `velocimeter` de MuJoCo
- validacion de la salida ONNX y de `last_action` en el lazo cerrado con Gazebo
- decidir si `wrist_scale_factor:=0.0` debe convertirse en el valor por defecto o mantenerse solo como recomendacion de lanzamiento

## Siguientes pasos recomendados

- integrar el paquete en un launch junto a Gazebo y el bridge
