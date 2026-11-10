#!/usr/bin/env bash
# Generates the 25 roadmap instances of each setting with make_roadmaps.
# Usage: experiments/make_instances.sh [setting ...]   (default: paper settings)
# A setting whose folder already holds all its instances is skipped.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
BIN="$ROOT/build/bin/make_roadmaps"
[ -x "$BIN" ] || { echo "build the project first (see README)" >&2; exit 1; }

settings=("$@")
[ ${#settings[@]} -gt 0 ] || mapfile -t settings < <(grep -v '^#' experiments/paper_settings.txt | awk 'NF {print $1}')

for s in "${settings[@]}"; do
  config="experiments/configs/$s.yaml"
  [ -f "$config" ] || { echo "no config for $s" >&2; exit 1; }
  dir="experiments/roadmaps/$s"
  n=$(awk '/^Iterations:/ {print $2}' "$config")
  if [ -d "$dir" ] && [ "$(ls "$dir" | wc -l)" -ge "$n" ]; then
    echo "$s: instances already there"
    continue
  fi
  echo "$s: generating $n instances"
  mkdir -p "$dir"
  "$BIN" "$config"
done
