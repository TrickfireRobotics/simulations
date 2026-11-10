#!/usr/bin/env bash

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
model="${1:-}"

case "${model}" in
arm | chassis | rover) ;;
*)
    echo "usage: $0 <arm|chassis|rover> [urdf-viz options]" >&2
    exit 2
    ;;
esac

shift

package_name="${model}_description"
package_dir="${workspace_dir}/gazebo/${package_name}"
source_file="${package_dir}/urdf/${model}.urdf"
ros_distro="${ROS_DISTRO:-jazzy}"

set -eo pipefail
source "/opt/ros/${ros_distro}/setup.bash"
set -u

cd "${workspace_dir}"
colcon build --base-paths gazebo --packages-select "${package_name}"
set +u
source "${workspace_dir}/install/setup.bash"
set -u

unset WAYLAND_DISPLAY
export WINIT_UNIX_BACKEND=x11

urdf_file="$(mktemp --suffix=.urdf)"
trap 'rm -f "${urdf_file}"' EXIT
xacro "${source_file}" >"${urdf_file}"

urdf-viz "${urdf_file}" --package-path "${package_name}=${package_dir}" "$@"
