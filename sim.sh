#!/usr/bin/env bash
# ============================================================================
# sim.sh  --  build / launch / stop any robot simulation in this repo.
#
#   ./sim.sh                                   # list robots and whether they're built
#   ./sim.sh <robot> build [pkg ...]           # colcon build that robot's workspace
#   ./sim.sh all build                         # build every workspace
#   ./sim.sh <robot> [gpu] [file.launch.py] [arg:=value ...]
#                                              # launch (default launch file if none given)
#   ./sim.sh stop                              # kill leftover Gazebo servers + sim nodes
#
# Targets: moz1 (spiritai_moz1), g1 (galbot_g1), rby1 (rainbowrobotics_rby1),
#          cell (hmlv_cell: the jerry-can transfer cell, robot:=moz1|g1)
#
#   ./sim.sh g1                                # Galbot G1, headless Gazebo + RViz
#   ./sim.sh rby1 gui:=true                    # RB-Y1 with the Gazebo GUI too
#   ./sim.sh moz1 sim_gazebo_jerrycan.launch.py
#   ./sim.sh cell robot:=g1                    # transfer demo with the Galbot G1
#
# Each is its own colcon workspace (Moz1 and the cell run Gazebo Fortress, G1 and
# RB-Y1 Gazebo Harmonic), so this script runs the launch in a subshell with only
# that workspace sourced. For multi-terminal work (demos, ros2 topic ...),
# `source <dir>/source_sim.sh` in each terminal instead.
# ============================================================================

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# robot -> "dir package default_launch"
declare -A ROBOTS=(
    [moz1]="spiritai_moz1 moz1_sim_gazebo sim_gazebo.launch.py"
    [g1]="galbot_g1 galbot_g1_gazebo sim.launch.py"
    [rby1]="rainbowrobotics_rby1 rby1_gazebo rby1_sim.launch.py"
    [cell]="hmlv_cell hmlv_cell_gazebo transfer.launch.py"
)
ORDER=(moz1 g1 rby1 cell)     # build order: cell overlays moz1

usage() {
    sed -n '3,23p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    echo
    for r in "${ORDER[@]}"; do
        read -r dir pkg launch <<<"${ROBOTS[$r]}"
        built="not built"; [[ -f "$REPO/$dir/install/setup.bash" ]] && built="built"
        printf '  %-5s %-22s %-18s %s\n' "$r" "$dir/" "$pkg" "($built)"
    done
}

resolve() {
    case "$1" in
        moz1|spiritai_moz1)               echo moz1 ;;
        g1|galbot|galbot_g1)              echo g1 ;;
        rby1|rb-y1|rainbowrobotics_rby1)  echo rby1 ;;
        cell|hmlv_cell)                   echo cell ;;
        *) return 1 ;;
    esac
}

# Drop any ROS / colcon / Gazebo environment inherited from the calling shell, so
# a Moz1 (Fortress) env can't leak into a Harmonic robot's launch or vice versa.
clean_env() {
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH \
          ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION GZ_VERSION \
          GZ_SIM_RESOURCE_PATH GZ_SIM_SYSTEM_PLUGIN_PATH \
          IGN_GAZEBO_RESOURCE_PATH IGN_GAZEBO_SYSTEM_PLUGIN_PATH
    local keep=() p
    IFS=: read -ra parts <<<"${LD_LIBRARY_PATH:-}"
    for p in "${parts[@]}"; do
        [[ -z "$p" || "$p" == /opt/ros/* || "$p" == *gz_fortress_ws* || "$p" == "$REPO"/* ]] || keep+=("$p")
    done
    LD_LIBRARY_PATH="$(IFS=:; echo "${keep[*]}")"; export LD_LIBRARY_PATH
}

source_robot() {  # $1 = dir, rest = source_sim.sh flags
    local dir="$1"; shift
    clean_env
    # shellcheck disable=SC1090
    source "$REPO/$dir/source_sim.sh" "$@" >/dev/null || return 1
    [[ -f "$REPO/$dir/install/setup.bash" ]] || { echo "❌ $dir is not built — run: ./sim.sh ${r} build"; return 1; }
}

descendants() { local c; for c in $(pgrep -P "$1"); do echo "$c"; descendants "$c"; done; }

stop_all() {
    local pids=() launch
    # Patterns are anchored at the start of the command line so they only match the
    # processes themselves, not a shell or editor that merely mentions them.
    # ros2 launch processes of this repo's packages, and everything they started
    for launch in $(pgrep -f '^\S*python3? \S*/ros2 launch (moz1_|galbot_|rby1_|hmlv_cell_)'); do
        pids+=("$launch" $(descendants "$launch"))
    done
    # Gazebo servers/GUIs (Fortress "ign gazebo", Harmonic "gz sim", both also via
    # "/bin/sh -c ruby $(which gz) sim") and orphaned nodes from this repo's installs
    pids+=($(pgrep -f '^(/bin/sh -c )?(\S*ruby )?(\S*/)?(ign|gz|\$\(which (ign|gz)\)) (gazebo|sim)( |$)'))
    pids+=($(pgrep -f "^(\S*python3? )?$REPO/[^ ]*/install/"))
    mapfile -t pids < <(printf '%s\n' "${pids[@]}" | grep -vx -e "$$" -e "$PPID" -e '' | sort -un)
    if (( ${#pids[@]} == 0 )); then echo "nothing to stop"; return 0; fi
    ps -o pid=,comm=,args= -p "${pids[@]}" | cut -c1-110 || true
    kill -TERM "${pids[@]}" 2>/dev/null || true
    sleep 3
    kill -KILL "${pids[@]}" 2>/dev/null || true
    echo "stopped ${#pids[@]} process(es)"
}

[[ $# -eq 0 || "$1" == -h || "$1" == --help || "$1" == help ]] && { usage; exit 0; }
[[ "$1" == stop ]] && { stop_all; exit 0; }

if [[ "$1" == all ]]; then
    [[ "${2:-}" == build ]] || { echo "usage: ./sim.sh all build"; exit 1; }
    for r in "${ORDER[@]}"; do
        read -r dir _ _ <<<"${ROBOTS[$r]}"
        ( clean_env; source "$REPO/$dir/source_sim.sh" build )
    done
    exit 0
fi

r="$(resolve "$1")" || { echo "unknown robot '$1'"; usage; exit 1; }
shift
read -r dir pkg launch <<<"${ROBOTS[$r]}"

if [[ "${1:-}" == build ]]; then
    shift
    clean_env
    source "$REPO/$dir/source_sim.sh" build "$@"
    exit 0
fi

flags=()
[[ "${1:-}" == gpu ]] && { flags+=(gpu); shift; }
[[ "${1:-}" == *.launch.py ]] && { launch="$1"; shift; }

source_robot "$dir" "${flags[@]}" || exit 1
echo "▶ ros2 launch $pkg $launch $*"
exec ros2 launch "$pkg" "$launch" "$@"
