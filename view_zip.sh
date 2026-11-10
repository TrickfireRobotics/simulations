#!/usr/bin/env bash

set -euo pipefail

zip_path="${1:-}"
if [[ -z "${zip_path}" || ! -f "${zip_path}" ]]; then
    echo "usage: $0 <onshape-export.zip> [urdf-viz options]" >&2
    exit 2
fi

shift
zip_path="$(realpath "${zip_path}")"

while IFS= read -r entry; do
    case "${entry}" in
    /* | .. | ../* | */../*)
        echo "refusing unsafe archive entry: ${entry}" >&2
        exit 1
        ;;
    esac
done < <(unzip -Z1 "${zip_path}")

extract_dir="$(mktemp -d)"
trap 'rm -rf "${extract_dir}"' EXIT
unzip -q "${zip_path}" -d "${extract_dir}"

mapfile -t urdf_files < <(find "${extract_dir}" -type f -path '*/urdf/*.urdf' | sort)
if (( ${#urdf_files[@]} != 1 )); then
    echo "expected one URDF under urdf/ in ${zip_path}, found ${#urdf_files[@]}" >&2
    exit 1
fi

urdf_file="${urdf_files[0]}"
package_dir="$(dirname "$(dirname "${urdf_file}")")"
package_name="$(basename "${package_dir}")"

unset WAYLAND_DISPLAY
export WINIT_UNIX_BACKEND=x11

urdf-viz "${urdf_file}" --package-path "${package_name}=${package_dir}" "$@"
