#!/usr/bin/env bash
# ============================================================================
# source_sim.sh  --  one-command env for the Moz1 simulation.
#
#   source source_sim.sh                # source the workspace
#   source source_sim.sh build          # colcon build, then source
#   source source_sim.sh gpu            # + route Gazebo rendering to the NVIDIA dGPU
#   source source_sim.sh isaac          # + DDS env for the Isaac Sim backend
#   source source_sim.sh build gpu      # build, source, offload — the full first-run line
#   source source_sim.sh build moz1_sim_gazebo    # build one package only
#
# Flags (any order; anything else is treated as a package name for `build`):
#   build   colcon build --symlink-install before sourcing
#   gpu     also source gpu_nvidia.sh — PRIME offload so the gz GUI + gpu_lidar
#           render on the NVIDIA card (REQUIRED for Nav2/SLAM on a hybrid laptop).
#   isaac   export ROS_DOMAIN_ID=33 + FastRTPS defaults so this host and the
#           Isaac Sim container discover each other. Not needed for Gazebo.
#
# Unlike the full SpiritAI checkout, everything here lives in ONE workspace
# (moz1_description, moz1_moveit_config, moz1_navigation_bringup and the
# moz1_sim_* packages), so there are no overlays to order and no SPIRITAI_DIR.
#
# MUST be sourced (not executed) so the environment persists in your shell.
# ============================================================================

# -- must be sourced ---------------------------------------------------------
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "⚠️  Source this, don't execute it:   source source_sim.sh"
    exit 1
fi

_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# -- parse flags out of the args; the rest are build packages -----------------
_DO_BUILD=""; _DO_GPU=""; _DO_ISAAC=""; _PKGS=()
for _a in "$@"; do
    case "$_a" in
        build)       _DO_BUILD=1 ;;
        gpu|nvidia)  _DO_GPU=1 ;;
        isaac)       _DO_ISAAC=1 ;;
        *)           _PKGS+=("$_a") ;;
    esac
done

echo "🛰️  Moz1 simulation env   (ws = ${_WS})"

# -- ROS 2 base --------------------------------------------------------------
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
    echo "   ✅ ROS 2 Humble"
else
    echo "   ❌ /opt/ros/humble/setup.bash not found — is ROS 2 Humble installed?"
    return 1 2>/dev/null || exit 1
fi

# -- optional build ----------------------------------------------------------
if [[ -n "${_DO_BUILD}" ]]; then
    echo "🔨 colcon build (--symlink-install)  ${_PKGS[*]:-<all packages>}"
    ( cd "${_WS}" && colcon build --symlink-install \
        ${_PKGS[*]:+--packages-select "${_PKGS[@]}"} ) \
        || { echo "   ❌ build failed"; return 1 2>/dev/null || exit 1; }
fi

# -- this workspace ----------------------------------------------------------
if [[ -f "${_WS}/install/setup.bash" ]]; then
    source "${_WS}/install/setup.bash"
    echo "   ✅ moz1 simulation workspace"
else
    echo "   ⚠️  ${_WS}/install/setup.bash not found — run: source source_sim.sh build"
fi

# -- optional NVIDIA offload (hybrid-laptop specific, opt-in) -----------------
if [[ -n "${_DO_GPU}" ]]; then
    if [[ -f "${_WS}/gpu_nvidia.sh" ]]; then
        source "${_WS}/gpu_nvidia.sh"
    else
        echo "   ⚠️  gpu: ${_WS}/gpu_nvidia.sh not found"
    fi
fi

# -- optional Isaac DDS env --------------------------------------------------
# Gazebo needs none of this (it is in-process on this host). Isaac Sim usually
# runs in a container, so both sides must share a domain and use plain FastDDS
# discovery. We deliberately do NOT set FASTRTPS_DEFAULT_PROFILES_FILE: the
# real-robot FastDDS profiles pin traffic to the robot's 172.16.0.x subnet,
# which is wrong for reaching the sim. If host and container don't discover each
# other, run Isaac with host networking and keep ROS_DOMAIN_ID aligned.
if [[ -n "${_DO_ISAAC}" ]]; then
    export ROS_DOMAIN_ID=33
    export ROS_LOCALHOST_ONLY=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    echo "   ✅ Isaac DDS: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  RMW=${RMW_IMPLEMENTATION}  LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
fi

unset _DO_BUILD _DO_GPU _DO_ISAAC _PKGS _a

echo "✨ Ready.  Launch e.g.:"
echo "   ros2 launch moz1_sim_gazebo sim_gazebo.launch.py          # Gazebo, robot only"
echo "   ros2 launch moz1_sim_gazebo sim_gazebo_moveit.launch.py   # + MoveIt (arms)"
echo "   ros2 launch moz1_sim_gazebo sim_gazebo_nav.launch.py      # + Nav2 / SLAM"
echo "   ros2 launch moz1_sim_gazebo sim_gazebo_full.launch.py     # BOTH in one sim"
