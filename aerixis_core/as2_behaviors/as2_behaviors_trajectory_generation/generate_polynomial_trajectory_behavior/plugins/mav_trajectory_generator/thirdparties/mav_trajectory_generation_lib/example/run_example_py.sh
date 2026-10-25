#!/bin/bash
# Launch the Python mav_trajectory_generation_cpp example using the package
# compiled in build/.
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$( cd "${SCRIPT_DIR}/.." && pwd )"
cd "${SCRIPT_DIR}"

PYTHON_PACKAGE_DIR="${PROJECT_DIR}/build/pybind/python"
PYTHON_EXAMPLE="${SCRIPT_DIR}/run_example.py"
SIM_CONFIG="config_example.yaml"
TRAJECTORY_CONFIG="config_trajectory.yaml"
OUTPUT_FILE="trajectory_logs/trajectory_py.csv"

if [ ! -d "${PYTHON_PACKAGE_DIR}/mav_trajectory_generation_py" ]; then
  echo "Python package not found at ${PYTHON_PACKAGE_DIR}/mav_trajectory_generation_py." >&2
  echo "Build the project first, e.g.:" >&2
  echo "  cmake -S ${PROJECT_DIR} -B ${PROJECT_DIR}/build -DBUILD_EXAMPLES=ON -DBUILD_PYBIND=ON" >&2
  echo "  cmake --build ${PROJECT_DIR}/build -j" >&2
  exit 1
fi

rm -f "${OUTPUT_FILE}"
PYTHONPATH="${PYTHON_PACKAGE_DIR}${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 "${PYTHON_EXAMPLE}" \
    -c "${SIM_CONFIG}" \
    -d "${TRAJECTORY_CONFIG}" \
    -f "${OUTPUT_FILE}"

if [ -f "${OUTPUT_FILE}" ]; then
  python3 plot_results.py \
    -f "${OUTPUT_FILE}" \
    -c "${SIM_CONFIG}" \
    --vmax 4.0
fi
