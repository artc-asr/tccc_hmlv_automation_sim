#!/usr/bin/env bash
# ============================================================================
# source_sim.sh  --  one-command env for the RB-Y1 simulation (Gazebo Harmonic).
#
#   source source_sim.sh                # source the workspace
#   source source_sim.sh build          # colcon build, then source
#   source source_sim.sh gpu            # + route Gazebo rendering to the NVIDIA dGPU
#   source source_sim.sh build rby1_gazebo        # build one package only
#
# Run from rainbowrobotics_rby1/ (or use ../sim.sh rby1 ... from the repo root).
#
# The build exports GZ_VERSION=harmonic: without it the vendored gz_ros2_control
# (src/gz_ros2_control -> ../../third_party/gz_ros2_control) silently builds
# against Gazebo Fortress and its plugin will not load in Harmonic.
#
# Use a fresh terminal: this workspace can't share a shell with the Moz1 one
# (Fortress bridge overlay) or with another robot's workspace.
#
# MUST be sourced (not executed) so the environment persists in your shell.
# ============================================================================

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "⚠️  Source this, don't execute it:   source source_sim.sh"
    exit 1
fi

_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_REPO="$(cd "${_WS}/.." && pwd)"

_DO_BUILD=""; _DO_GPU=""; _PKGS=()
for _a in "$@"; do
    case "$_a" in
        build)       _DO_BUILD=1 ;;
        gpu|nvidia)  _DO_GPU=1 ;;
        *)           _PKGS+=("$_a") ;;
    esac
done

echo "🛰️  RB-Y1 simulation env   (ws = ${_WS})"

if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
    echo "   ✅ ROS 2 Humble"
else
    echo "   ❌ /opt/ros/humble/setup.bash not found — is ROS 2 Humble installed?"
    return 1 2>/dev/null || exit 1
fi

if [[ ":${AMENT_PREFIX_PATH}:" == *"gz_fortress_ws"* ]]; then
    echo "   ⚠️  this shell already has the Moz1 Fortress overlay — use a fresh terminal"
fi

if [[ -n "${_DO_BUILD}" ]]; then
    echo "🔨 colcon build (GZ_VERSION=harmonic, Release)  ${_PKGS[*]:-<all packages>}"
    ( cd "${_WS}" && GZ_VERSION=harmonic colcon build --symlink-install \
        ${_PKGS[*]:+--packages-select "${_PKGS[@]}"} \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF ) \
        || { echo "   ❌ build failed"; return 1 2>/dev/null || exit 1; }
fi

if [[ -f "${_WS}/install/setup.bash" ]]; then
    source "${_WS}/install/setup.bash"
    echo "   ✅ rainbowrobotics_rby1 workspace"
else
    echo "   ⚠️  ${_WS}/install/setup.bash not found — run: source source_sim.sh build"
fi

if [[ -n "${_DO_GPU}" ]]; then
    source "${_REPO}/gpu_nvidia.sh"
fi

unset _WS _REPO _DO_BUILD _DO_GPU _PKGS _a

echo "✨ Ready.  Launch e.g.:"
echo "   ros2 launch rby1_gazebo rby1_sim.launch.py              # headless Gazebo + RViz"
echo "   ros2 launch rby1_gazebo rby1_sim.launch.py gui:=true    # + Gazebo GUI"
echo "   ros2 run rby1_gazebo rby1_demo.py                       # (2nd terminal) ready pose, head, grippers, drive"
