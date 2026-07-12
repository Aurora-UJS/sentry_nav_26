#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." >/dev/null 2>&1 && pwd)"
IMAGE="${SENTRY_NAV_IMAGE:-localhost/sentry-nav-26:jazzy}"
CONTAINER_USER="${SENTRY_NAV_CONTAINER_USER:-ros}"
UBUNTU_MIRROR="${SENTRY_NAV_UBUNTU_MIRROR:-http://mirrors.tuna.tsinghua.edu.cn/ubuntu}"
ROS_APT_MIRROR="${SENTRY_NAV_ROS_APT_MIRROR:-}"
WORKDIR="/workspace/sentry_nav_26"

usage() {
    cat <<'USAGE'
Usage: ./scripts/podman-dev.sh <command> [args...]

Commands:
  build-image   Build the ROS 2 Jazzy development image
  shell         Open an interactive shell in the dev container
  submodules    Fetch/update simulation/LIO submodules from inside the container
  deps          Run rosdep for packages under src/
  build         Run colcon build --symlink-install
  test          Run colcon test
  zenoh         Start the rmw_zenoh router
  bringup       Start the simulation/navigation launch without embedded teleop
  teleop        Start teleop_twist_keyboard in the current terminal
  exec <cmd>    Run an arbitrary command in the dev container

Environment:
  SENTRY_NAV_IMAGE           Override image tag
  SENTRY_NAV_CONTAINER_USER  Override container username
  ROS_DOMAIN_ID              Forwarded into the container, default 0
  RMW_IMPLEMENTATION         Forwarded into the container, default rmw_zenoh_cpp
  SENTRY_NAV_UBUNTU_MIRROR   Ubuntu apt mirror used at image build time
  SENTRY_NAV_ROS_APT_MIRROR  Optional ROS apt mirror used at image build time
USAGE
}

image_exists() {
    podman image exists "${IMAGE}"
}

build_image() {
    podman build \
        --network host \
        --format docker \
        --tag "${IMAGE}" \
        --file "${REPO_ROOT}/Containerfile" \
        --build-arg "USERNAME=${CONTAINER_USER}" \
        --build-arg "USER_UID=$(id -u)" \
        --build-arg "USER_GID=$(id -g)" \
        --build-arg "UBUNTU_MIRROR=${UBUNTU_MIRROR}" \
        --build-arg "ROS_APT_MIRROR=${ROS_APT_MIRROR}" \
        "${REPO_ROOT}"
}

ensure_image() {
    if ! image_exists; then
        build_image
    fi
}

allow_x11_for_current_user() {
    if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
        xhost "+SI:localuser:$(id -un)" >/dev/null 2>&1 || true
    fi
}

run_container() {
    ensure_image
    allow_x11_for_current_user

    local name="sentry-nav-dev-$$"
    local args=(
        run
        --rm
        -it
        --name "${name}"
        --userns keep-id
        --network host
        --ipc host
        --security-opt label=disable
        --hostname sentry-nav-dev
        --workdir "${WORKDIR}"
        --env "QT_X11_NO_MITSHM=1"
        --env "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
        --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
        --env "TERM=${TERM:-xterm-256color}"
        --volume "${REPO_ROOT}:${WORKDIR}:rw"
    )

    if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
        args+=(--env DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:rw)
    fi

    if [ -n "${WAYLAND_DISPLAY:-}" ]; then
        args+=(--env WAYLAND_DISPLAY)
    fi

    if [ -n "${XDG_RUNTIME_DIR:-}" ] && [ -d "${XDG_RUNTIME_DIR}" ]; then
        args+=(--env XDG_RUNTIME_DIR --volume "${XDG_RUNTIME_DIR}:${XDG_RUNTIME_DIR}:rw")
    fi

    if [ -n "${XAUTHORITY:-}" ] && [ -r "${XAUTHORITY}" ]; then
        args+=(--env "XAUTHORITY=${XAUTHORITY}" --volume "${XAUTHORITY}:${XAUTHORITY}:ro")
    fi

    if [ -e /dev/dri ]; then
        args+=(--device /dev/dri --group-add keep-groups)
    fi

    if [ -f "${HOME}/.gitconfig" ]; then
        args+=(--volume "${HOME}/.gitconfig:/home/${CONTAINER_USER}/.gitconfig:ro")
    fi

    args+=("${IMAGE}")
    podman "${args[@]}" "$@"
}

command="${1:-shell}"
case "${command}" in
    -h|--help|help)
        usage
        ;;
    build-image)
        build_image
        ;;
    shell)
        run_container bash
        ;;
    submodules)
        run_container git submodule update --init --recursive src/rm_sim_26 src/small_point_lio
        ;;
    deps)
        run_container bash -lc 'rosdep update --rosdistro "${ROS_DISTRO:-jazzy}" || true; sudo apt-get update; rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS_DISTRO:-jazzy}"'
        ;;
    build)
        # RelWithDebInfo: 默认 colcon 构建不带任何 -O 标志 (-O0)，A* 单次
        # init 扩展实测 35-56ms、ESDF/MINCO 全线拖慢 10-30x
        run_container colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
        ;;
    test)
        run_container colcon test --event-handlers console_direct+
        ;;
    zenoh)
        run_container ros2 run rmw_zenoh_cpp rmw_zenohd
        ;;
    bringup)
        run_container ros2 launch sentry_bringup bringup.launch.py start_teleop:=false
        ;;
    teleop)
        run_container ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true
        ;;
    exec)
        shift
        if [ "$#" -eq 0 ]; then
            usage
            exit 2
        fi
        run_container "$@"
        ;;
    *)
        run_container "$@"
        ;;
esac
