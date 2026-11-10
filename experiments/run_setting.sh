#!/usr/bin/env bash
# Runs every planner of a setting on its 25 instances in a single process
# and writes experiments/results/<setting>.csv. Instances are generated first
# if missing. Simple, but slow and memory hungry on the 700-agent settings:
# use run_campaign.sh there.
# Usage: experiments/run_setting.sh <setting>
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
s=${1:?usage: run_setting.sh <setting>}
"$ROOT/experiments/make_instances.sh" "$s"
mkdir -p experiments/results
"$ROOT/build/bin/planning_incrementally" "experiments/configs/$s.yaml" \
  > "experiments/results/$s.csv" 2> "experiments/results/$s.log"
echo "results: experiments/results/$s.csv"
