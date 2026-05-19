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
- subscribe a `/cmd_vel`
- subscribe a `/g1/last_action`
- subscribe opcionalmente a `/g1/pelvis_twist`
- publica la observacion completa en `/g1/observation` como `std_msgs/msg/Float64MultiArray`

El nodo `policy_inference`:

- subscribe a `/g1/observation`
- carga la politica ONNX
- publica `last_action` en `/g1/last_action`
- publica objetivos articulares en `/g1/cmd_pos/<joint_name>`

La semantica del control replica la referencia MuJoCo:

- `raw_action = policy(obs)`
- `last_action = raw_action`
- `joint_target = default_joint_pos + raw_action * action_scale`

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

## Temas usados por defecto

Entradas:

- `/g1/joint_states`
- `/g1/imu/pelvis`
- `/g1/pelvis/odometry`
- `/g1/pelvis_twist`
- `/cmd_vel`
- `/g1/last_action`
- `/g1/observation`

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

## Estado de validacion

Validado hasta ahora:

- sintaxis Python del paquete
- coherencia interna del vector `obs[99]`
- uso del mismo orden articular que la politica de MuJoCo
- build del paquete `policy_control`

Pendiente:

- ejecucion real con la simulacion
- validacion numerica contra MuJoCo
- validacion de `/g1/pelvis/odometry` como sustituto del `velocimeter` de MuJoCo
- validacion de la salida ONNX y de `last_action` en el lazo cerrado con Gazebo

## Siguientes pasos recomendados

- integrar el paquete en un launch junto a Gazebo y el bridge
- validar si hace falta anadir `onnxruntime` al Dockerfile del proyecto
