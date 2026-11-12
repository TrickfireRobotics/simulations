#!/usr/bin/env bash

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
model="${1:-}"

case "${model}" in
arm)
    package_name="arm_description"
    source_name="arm.urdf"
    build_packages=(arm_description)
    ;;
viator_chassis)
    package_name="viator_description"
    source_name="viator_chassis.urdf.xacro"
    build_packages=(viator_description)
    ;;
viator)
    package_name="viator_description"
    source_name="viator.urdf.xacro"
    build_packages=(arm_description viator_description)
    ;;
*)
    echo "usage: $0 <arm|viator_chassis|viator> [urdf-viz options]" >&2
    exit 2
    ;;
esac

shift

package_dir="${workspace_dir}/gazebo/${package_name}"
source_file="${package_dir}/urdf/${source_name}"
ros_distro="${ROS_DISTRO:-jazzy}"

set -eo pipefail
source "/opt/ros/${ros_distro}/setup.bash"
set -u

cd "${workspace_dir}"
colcon build --base-paths gazebo --packages-select "${build_packages[@]}"
set +u
source "${workspace_dir}/install/setup.bash"
set -u

unset WAYLAND_DISPLAY
export WINIT_UNIX_BACKEND=x11

urdf_file="$(mktemp --suffix=.urdf)"
trap 'rm -f "${urdf_file}"' EXIT
xacro "${source_file}" >"${urdf_file}"

package_paths=(--package-path "${package_name}=${package_dir}")
if [[ "${model}" == "viator" ]]; then
    package_paths+=(--package-path "arm_description=${workspace_dir}/gazebo/arm_description")
fi

urdf-viz "${urdf_file}" "${package_paths[@]}" "$@"
