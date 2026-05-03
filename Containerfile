ARG BASE_IMAGE=docker.io/osrf/ros:jazzy-desktop-full
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=jazzy
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
ARG UBUNTU_MIRROR=http://mirrors.tuna.tsinghua.edu.cn/ubuntu
ARG ROS_APT_MIRROR=

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    QT_X11_NO_MITSHM=1

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN bash -lc 'set -eu; \
    if [ -n "${UBUNTU_MIRROR}" ]; then \
        sed -i "s|http://archive.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g; s|http://security.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g" /etc/apt/sources.list /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources 2>/dev/null || true; \
    fi; \
    if [ -n "${ROS_APT_MIRROR}" ]; then \
        sed -i "s|http://packages.ros.org/ros2/ubuntu|${ROS_APT_MIRROR}|g; s|https://packages.ros.org/ros2/ubuntu|${ROS_APT_MIRROR}|g" /etc/apt/sources.list /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources 2>/dev/null || true; \
    fi'

RUN bash -lc 'apt-get update && apt-get install -y --no-install-recommends \
    -o Acquire::Retries=5 \
    -o Acquire::http::Timeout=60 \
    -o Acquire::https::Timeout=60 \
    bash-completion \
    build-essential \
    cmake \
    git \
    iputils-ping \
    less \
    libboost-all-dev \
    libceres-dev \
    libeigen3-dev \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libnlopt-cxx-dev \
    libnlopt-dev \
    libopencv-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    mesa-utils \
    nano \
    pkg-config \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    sudo \
    vim \
    zsh \
    ros-${ROS_DISTRO}-ament-cmake-auto \
    ros-${ROS_DISTRO}-gz-sim-vendor \
    ros-${ROS_DISTRO}-gz-tools-vendor \
    ros-${ROS_DISTRO}-laser-geometry \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-tf2-sensor-msgs \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*'

RUN set -eu; \
    if ! getent group "${USER_GID}" >/dev/null; then \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi; \
    if id -u "${USERNAME}" >/dev/null 2>&1; then \
        usermod -o -u "${USER_UID}" -g "${USER_GID}" -d "/home/${USERNAME}" -s /bin/bash "${USERNAME}"; \
    else \
        useradd -o -u "${USER_UID}" -g "${USER_GID}" -m -s /bin/bash "${USERNAME}"; \
    fi; \
    mkdir -p "/home/${USERNAME}"; \
    chown -R "${USER_UID}:${USER_GID}" "/home/${USERNAME}"; \
    echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"; \
    rosdep init || true

COPY containers/entrypoint.sh /ros_entrypoint.sh
RUN bash -lc 'chmod +x /ros_entrypoint.sh \
    && { \
        echo "source /opt/ros/${ROS_DISTRO}/setup.bash"; \
        echo "[ -f /workspace/sentry_nav_26/install/setup.bash ] && source /workspace/sentry_nav_26/install/setup.bash"; \
    } >> /home/${USERNAME}/.bashrc \
    && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.bashrc'

USER ${USERNAME}
WORKDIR /workspace/sentry_nav_26

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
