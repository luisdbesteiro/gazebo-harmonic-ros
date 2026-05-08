FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Herramientas básicas
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
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

# Crear carpeta de trabajo
WORKDIR /workspace

# Auto-source de ROS en bash
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/ros2_gz_ws/install/setup.bash ]; then source /workspace/ros2_gz_ws/install/setup.bash; fi" >> /root/.bashrc

CMD ["bash"]
