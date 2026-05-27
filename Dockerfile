FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Herramientas básicas
RUN apt-get update && apt-get install -y \
    curl \
    ca-certificates \
    gnupg \
    lsb-release \
    mesa-utils \
    nano \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Añadir repositorio oficial de Gazebo / OSRF dentro del contenedor
RUN curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# Instalar Gazebo Harmonic + integración ROS 2 Humble/Harmonic dentro del contenedor
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    ros-humble-ros-gzharmonic \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# ONNX Runtime C/C++ para el nodo integrado de policy_control.
ARG ONNXRUNTIME_VERSION=1.18.1
ARG TARGETARCH
RUN set -eux; \
    case "${TARGETARCH:-amd64}" in \
        amd64) ORT_ARCH="x64" ;; \
        arm64) ORT_ARCH="aarch64" ;; \
        *) echo "Arquitectura no soportada para ONNX Runtime: ${TARGETARCH}" >&2; exit 1 ;; \
    esac; \
    curl -L "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-${ORT_ARCH}-${ONNXRUNTIME_VERSION}.tgz" \
        -o /tmp/onnxruntime.tgz; \
    tar -xzf /tmp/onnxruntime.tgz -C /opt; \
    mv "/opt/onnxruntime-linux-${ORT_ARCH}-${ONNXRUNTIME_VERSION}" /opt/onnxruntime; \
    echo "/opt/onnxruntime/lib" > /etc/ld.so.conf.d/onnxruntime.conf; \
    ldconfig; \
    rm /tmp/onnxruntime.tgz

ENV ONNXRUNTIME_ROOT=/opt/onnxruntime

# Dependencias Python del proyecto dentro del contenedor
RUN python3 -m pip install --no-cache-dir \
    onnxruntime

# Crear carpeta de trabajo
WORKDIR /workspace

# Auto-source de ROS en bash
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/ros2_ws/install/setup.bash ]; then source /workspace/ros2_ws/install/setup.bash; fi" >> /root/.bashrc

CMD ["bash"]
