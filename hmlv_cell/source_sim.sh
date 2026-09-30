#!/usr/bin/env bash
# ============================================================================
# source_sim.sh  --  env for the HMLV cell (Gazebo Fortress, any robot).
#
#   source source_sim.sh                # source the workspace
#   source source_sim.sh build          # colcon build, then source
#   source source_sim.sh gpu            # + route Gazebo rendering to the NVIDIA dGPU
#
# This workspace OVERLAYS spiritai_moz1/ (it uses moz1_sim_gazebo's Fortress
# plugins and the Moz1 packages, and the Fortress ros_gz bridge that Moz1's env
# finds), so that one is sourced first; build it before this one
# (../sim.sh all build does the order). The Galbot G1's description and MoveIt
# config are symlinked in from ../galbot_g1/src.
#
# Use a fresh terminal: not one with a Harmonic (galbot_g1 / rby1) env in it.
# MUST be sourced (not executed) so the environment persists in your shell.
# ============================================================================

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "⚠️  Source this, don't execute it:   source source_sim.sh"
    exit 1
fi

_CELL_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_CELL_BUILD=""; _CELL_FLAGS=(); _CELL_PKGS=()
for _a in "$@"; do
    case "$_a" in
        build)       _CELL_BUILD=1 ;;
        gpu|nvidia)  _CELL_FLAGS+=(gpu) ;;
        *)           _CELL_PKGS+=("$_a") ;;
    esac
done

# The Moz1 workspace + Fortress bridge underneath (its own messages trimmed). Sourced
# from a function: a bare `source` with no arguments would hand it OUR arguments
# (e.g. build) — inside the function it gets exactly the function's.
_cell_source_moz1() {
    source "${_CELL_WS}/../spiritai_moz1/source_sim.sh" "$@" \
        > >(grep -v "^   ros2 launch\|Launch e.g")
}
_cell_source_moz1 "${_CELL_FLAGS[@]}"
unset -f _cell_source_moz1
echo "🛰️  HMLV cell env   (ws = ${_CELL_WS})"

if [[ -n "${_CELL_BUILD}" ]]; then
    echo "🔨 colcon build (--symlink-install)  ${_CELL_PKGS[*]:-<all packages>}"
    ( cd "${_CELL_WS}" && colcon build --symlink-install \
        ${_CELL_PKGS[*]:+--packages-select "${_CELL_PKGS[@]}"} ) \
        || { echo "   ❌ build failed"; return 1 2>/dev/null || exit 1; }
fi

# local_setup only: setup.bash would re-source /opt/ros/humble and put the Harmonic
# ros_gz back in front of the Fortress bridge overlay.
if [[ -f "${_CELL_WS}/install/local_setup.bash" ]]; then
    source "${_CELL_WS}/install/local_setup.bash"
    echo "   ✅ hmlv_cell workspace"
else
    echo "   ⚠️  ${_CELL_WS}/install not found — run: source source_sim.sh build"
fi

# Gazebo transport on loopback: ign-transport binds its discovery to the network
# interface when a process starts, so a Wi-Fi address change mid-run (a DHCP renewal,
# a roam) left the processes started before and after unable to find each other —
# the duo demo's `ign topic` pose queries then got nothing ("no pose from Gazebo").
# Everything here is on this machine; set IGN_IP yourself to reach another one.
export IGN_IP="${IGN_IP:-127.0.0.1}"

unset _CELL_WS _CELL_BUILD _CELL_FLAGS _CELL_PKGS _a

echo "✨ Ready.  Launch e.g.:"
echo "   ros2 launch hmlv_cell_gazebo transfer.launch.py              # Moz1 (default)"
echo "   ros2 launch hmlv_cell_gazebo transfer.launch.py robot:=g1    # Galbot G1"
echo "   ros2 launch hmlv_cell_gazebo duo.launch.py                   # G1 + Moz1"
