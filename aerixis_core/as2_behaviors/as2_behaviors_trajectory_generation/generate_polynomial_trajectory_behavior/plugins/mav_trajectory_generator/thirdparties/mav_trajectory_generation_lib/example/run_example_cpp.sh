#!/bin/bash
# Launch the mav_trajectory_generation_cpp example: generate the trajectory,
# log it to CSV and plot the results. Paths are resolved relative to this
# script's directory (example/), so it can be invoked from anywhere.
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$( cd "${SCRIPT_DIR}/.." && pwd )"
cd "${SCRIPT_DIR}"

BINARY="${PROJECT_DIR}/build/example/mav_trajectory_generation_cpp_run_example"
SIM_CONFIG="config_example.yaml"
TRAJECTORY_CONFIG="config_trajectory.yaml"
OUTPUT_FILE="trajectory_logs/trajectory.csv"

if [ ! -x "${BINARY}" ]; then
  echo "Binary not found at ${BINARY}." >&2
  echo "Build the project first, e.g.:" >&2
  echo "  cmake -S ${PROJECT_DIR} -B ${PROJECT_DIR}/build -DBUILD_EXAMPLES=ON" >&2
  echo "  cmake --build ${PROJECT_DIR}/build -j" >&2
  exit 1
fi

rm -f "${OUTPUT_FILE}"
"${BINARY}" \
  -c "${SIM_CONFIG}" \
  -d "${TRAJECTORY_CONFIG}" \
  -f "${OUTPUT_FILE}"

if [ -f "${OUTPUT_FILE}" ]; then
  python3 plot_results.py \
    -f "${OUTPUT_FILE}" \
    -c "${SIM_CONFIG}" \
    --vmax 4.0
fi
