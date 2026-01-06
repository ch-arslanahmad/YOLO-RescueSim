#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: /opt/ros/jazzy/setup.bash not found. Is ROS 2 Jazzy installed?" >&2
  exit 1
fi

# ROS setup scripts are not always compatible with 'set -u' (nounset).
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

cmd=(colcon build --packages-select project --symlink-install)
if [[ $# -gt 0 ]]; then
  cmd+=("$@")
fi

log_file="$(mktemp -t colcon_project_build.XXXXXX.log)"

set +e
"${cmd[@]}" 2>&1 | tee "$log_file"
status=${PIPESTATUS[0]}
set -e

if [[ $status -eq 0 ]]; then
  exit 0
fi

# Common ament_cmake_python symlink-install failure:
# "failed to create symbolic link ... existing path cannot be removed: Is a directory"
if grep -q "failed to create symbolic link" "$log_file" && grep -q "Is a directory" "$log_file"; then
  echo
  echo "Detected stale symlink-install artifacts. Cleaning build/project and retrying..." >&2
  rm -rf build/project
  "${cmd[@]}"
  exit 0
fi

echo
echo "Build failed (see log: $log_file)" >&2
exit $status
