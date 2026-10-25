#!/usr/bin/env bash
# Link this checkout to the places the laptop launchers expect: ~/aerostack2 -> aerixis_core/
# (the AERIXIS core folder) and ~/custom_gcs -> custom_gcs/.
# Existing folders are never overwritten: a real folder already there is left alone and reported.
set -u
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for pair in "aerostack2:aerixis_core" "custom_gcs:custom_gcs"; do
  link="$HOME/${pair%%:*}"; source="$HERE/${pair##*:}"
  if [ -L "$link" ]; then
    ln -sfn "$source" "$link" && echo "updated link $link -> $source"
  elif [ -e "$link" ]; then
    echo "kept existing $link (not a link); move it away to use this checkout"
  else
    ln -s "$source" "$link" && echo "linked $link -> $source"
  fi
done
[ -f "$HERE/custom_gcs/config/fleet.json" ] || echo "note: no GCS fleet yet; add aircraft on the GCS Connect page"
echo "next: see README.md (build the Pi image, then ./start_field.sh)"
