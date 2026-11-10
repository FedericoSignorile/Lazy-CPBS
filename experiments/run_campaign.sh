#!/usr/bin/env bash
# Runs the experimental campaign in parallel: one process per
# (setting, method, instance), so that memory stays bounded (the 25 annotated
# roadmaps of a 700-agent setting together take several GB).
#
# Usage: experiments/run_campaign.sh [-j JOBS] [-o OUT_DIR] [setting ...]
#   default settings: experiments/paper_settings.txt, with their methods
#   default JOBS: number of cores / 2;  default OUT_DIR: experiments/results
#
# Environment:
#   METHODS       override the methods of every setting (space separated)
#   MEM_CAP_GB    kill a process above this RSS and rerun it alone at the end (6)
#   HANG_MIN      kill and retry a process silent for this many minutes (20)
#   MAX_TRIES     attempts per process (3)
#
# Output: OUT_DIR/<setting>.csv, one row per (n, instance, method), plus
# OUT_DIR/jobs/ with the raw output of every process and OUT_DIR/progress.log.
# Finished processes are skipped when the script is rerun.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PAR=$(( $(nproc) / 2 )); [ "$PAR" -ge 1 ] || PAR=1
OUT="$ROOT/experiments/results"
while getopts "j:o:" opt; do
  case $opt in
    j) PAR=$OPTARG ;;
    o) OUT=$(realpath -m "$OPTARG") ;;
    *) sed -n '2,20p' "$0"; exit 1 ;;
  esac
done
shift $((OPTIND - 1))

export ROOT OUT BIN="$ROOT/build/bin/planning_incrementally"
export MEM_CAP_GB=${MEM_CAP_GB:-6} HANG_MIN=${HANG_MIN:-20} MAX_TRIES=${MAX_TRIES:-3}
[ -x "$BIN" ] || { echo "build the project first (see README)" >&2; exit 1; }

ALL="CCBS CCBS_CTC PSIPP CPBS LazyCPBS Eager_ReplanList_EGT Eager_ReplanList Eager_Fixed_EGT"
MAIN="CCBS CCBS_CTC PSIPP CPBS LazyCPBS"

declare -A methods_of
if [ $# -gt 0 ]; then
  for s in "$@"; do methods_of[$s]=$ALL; done
else
  while read -r s kind; do
    [ "$kind" = main ] && methods_of[$s]=$MAIN || methods_of[$s]=$ALL
  done < <(grep -v '^#' experiments/paper_settings.txt | awk 'NF')
fi
[ -n "${METHODS:-}" ] && for s in "${!methods_of[@]}"; do methods_of[$s]=$METHODS; done

experiments/make_instances.sh "${!methods_of[@]}" || exit 1

mkdir -p "$OUT/jobs"
JOBS="$OUT/jobs.txt"; : > "$JOBS"
# 700-agent settings first: they are the longest, this packs the jobs better
for s in $(printf '%s\n' "${!methods_of[@]}" | sort -t_ -k2,2nr -k1,1); do
  n=$(awk '/^Iterations:/ {print $2}' "experiments/configs/$s.yaml")
  for m in ${methods_of[$s]}; do
    for k in $(seq 0 $((n - 1))); do echo "$s $m $k" >> "$JOBS"; done
  done
done
{
  echo "start: $(date -Is)  host: $(hostname)  jobs: $PAR  processes: $(wc -l < "$JOBS")"
  echo "commit: $(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo none)"
} >> "$OUT/progress.log"

# one attempt with watchdog; prints ok | hang | mem | exit<code>
attempt() {
  local cfg=$1 m=$2 k=$3 dir=$4 cap_kb=$5
  "$BIN" "$cfg" --instance "$k" --method "$m" > "$dir/out.csv" 2> "$dir/err.log" &
  local pid=$! last=-1 changed=$(date +%s) reason="" size now rss
  while kill -0 $pid 2>/dev/null; do
    sleep 5
    size=$(( $(stat -c %s "$dir/out.csv") + $(stat -c %s "$dir/err.log") ))
    now=$(date +%s)
    [ "$size" -ne "$last" ] && { last=$size; changed=$now; }
    if [ $((now - changed)) -ge $((HANG_MIN * 60)) ]; then reason=hang; kill $pid; break; fi
    if [ "$cap_kb" -gt 0 ]; then
      rss=$(ps -o rss= -p $pid 2>/dev/null | tr -d ' ')
      if [ -n "$rss" ] && [ "$rss" -gt "$cap_kb" ]; then reason=mem; kill $pid; break; fi
    fi
  done
  wait $pid; local rc=$?
  if [ -n "$reason" ]; then echo $reason; elif [ $rc -eq 0 ]; then echo ok; else echo exit$rc; fi
}

# merges the per-instance CSVs of a setting once all its processes are done
merge_setting() {
  local s=$1
  (
    flock 9
    local expected done_
    expected=$(grep -c "^$s " "$OUT/jobs.txt")
    done_=$(ls "$OUT/jobs/$s"/*/done 2>/dev/null | wc -l)
    [ "$done_" -lt "$expected" ] && exit 0
    python3 - "$OUT/jobs/$s" "$OUT/$s.csv" <<'PY'
import csv, glob, os, sys
rows, header = [], None
for f in sorted(glob.glob(os.path.join(sys.argv[1], "*", "out.csv"))):
    with open(f) as fh:
        r = csv.DictReader(fh)
        header = header or r.fieldnames
        rows += list(r)
rows.sort(key=lambda r: (int(r["n"]), int(r["instance"]), r["method"]))
with open(sys.argv[2], "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=header)
    w.writeheader()
    w.writerows(rows)
PY
    echo "$(date '+%F %T') MERGED $s -> $OUT/$s.csv" >> "$OUT/progress.log"
  ) 9> "$OUT/jobs/$s/.lock"
}

run_job() {
  local s=$1 m=$2 k=$3 cap_kb=$4
  local dir="$OUT/jobs/$s/${m}_$k" cfg="$ROOT/experiments/configs/$s.yaml"
  mkdir -p "$dir"
  [ -f "$dir/done" ] && { merge_setting "$s"; return; }
  local try=1 res t0
  t0=$(date +%s)
  while true; do
    res=$(attempt "$cfg" "$m" "$k" "$dir" "$cap_kb")
    [ "$res" = ok ] && break
    if [ "$res" = mem ]; then
      echo "$s $m $k" >> "$OUT/jobs_mem_requeue.txt"
      echo "$(date '+%F %T') REQUEUE $s $m $k: above ${MEM_CAP_GB} GB, rerun alone at the end" >> "$OUT/progress.log"
      return
    fi
    [ $try -ge "$MAX_TRIES" ] && break
    echo "$(date '+%F %T') RETRY $s $m $k: attempt $try failed ($res)" >> "$OUT/progress.log"
    try=$((try + 1))
  done
  [ "$res" != ok ] && echo "$(date '+%F %T') FAILED $s $m $k after $try attempts ($res)" >> "$OUT/progress.log"
  echo "$res $(( $(date +%s) - t0 ))s" > "$dir/done"
  echo "$(date '+%F %T') $s $m $k $res $(( $(date +%s) - t0 ))s" >> "$OUT/instances.log"
  merge_setting "$s"
}
export -f attempt merge_setting run_job

: > "$OUT/jobs_mem_requeue.txt"
xargs -a "$JOBS" -P "$PAR" -L 1 bash -c 'run_job "$0" "$1" "$2" $((MEM_CAP_GB * 1024 * 1024))'
if [ -s "$OUT/jobs_mem_requeue.txt" ]; then
  echo "$(date '+%F %T') PHASE2: $(wc -l < "$OUT/jobs_mem_requeue.txt") processes without memory cap, one at a time" >> "$OUT/progress.log"
  while read -r s m k; do run_job "$s" "$m" "$k" 0; done < "$OUT/jobs_mem_requeue.txt"
fi
echo "$(date '+%F %T') CAMPAIGN_FINISHED" >> "$OUT/progress.log"
