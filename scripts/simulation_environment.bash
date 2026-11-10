# Source in each simulation terminal after the ROS and workspace setup files.
# Keep graphics, audio and Python library settings inherited from the container.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    printf '%s\n' 'Use: source scripts/simulation_environment.bash' >&2
    exit 1
fi

export IGN_IP="${IGN_IP:-127.0.0.1}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../config" && pwd)/cyclonedds_simulation.xml}"
# The XML explicitly selects loopback multicast. ROS_LOCALHOST_ONLY=1 instead
# disables multicast in Humble and exhausts Cyclone's default participant slots.
export ROS_LOCALHOST_ONLY=0
