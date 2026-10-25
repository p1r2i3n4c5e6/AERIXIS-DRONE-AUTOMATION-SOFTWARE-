#!/usr/bin/env bash
# Link this checkout to the places the laptop launchers expect: ~/aerostack2 and ~/custom_gcs.
# Existing folders are never overwritten: a real folder already there is left alone and reported.
set -u
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for name in aerostack2 custom_gcs; do
  target="$HOME/$name"
  if [ -L "$target" ]; then
    ln -sfn "$HERE/$name" "$target" && echo "updated link $target -> $HERE/$name"
  elif [ -e "$target" ]; then
    echo "kept existing $target (not a link); move it away to use this checkout"
  else
    ln -s "$HERE/$name" "$target" && echo "linked $target -> $HERE/$name"
  fi
done
[ -f "$HERE/custom_gcs/config/fleet.json" ] || echo "note: no GCS fleet yet; add aircraft on the GCS Connect page"
echo "next: see README.md (build the Pi image, then ./start_field.sh)"
