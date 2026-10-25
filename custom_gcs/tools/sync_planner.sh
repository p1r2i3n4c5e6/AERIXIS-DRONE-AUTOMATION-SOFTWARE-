#!/usr/bin/env bash
# Copy the mission planner (pure Python, no ROS) from an AERIXIS core checkout into this GCS, so
# the GCS plans on a laptop that has no AS2 installed. Run it on purpose after the planner changes,
# then upload the same AERIXIS core version to the drones (they re-validate every plan onboard).
#
#   tools/sync_planner.sh [path/to/aerostack2]      (default ~/aerostack2)
set -euo pipefail
SRC="${1:-$HOME/aerostack2}/rescue_ws/src/as2_rescue_planner/as2_rescue_planner"
DST="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)/vendor/as2_rescue_planner"
[ -f "$SRC/__init__.py" ] || { echo "planner not found at $SRC" >&2; exit 1; }
rm -rf "$DST.new"; mkdir -p "$DST.new"
cp "$SRC"/*.py "$DST.new"/
( cd "$DST.new" && sha256sum *.py | sha256sum | cut -c1-12 ) > "$DST.new/VERSION"
rm -rf "$DST"; mv "$DST.new" "$DST"
echo "planner copied to $DST (version $(cat "$DST/VERSION"))"
