#!/usr/bin/env bash
set -e   # abort on any error

# 1) where to put your data
OUTPUT_BASE="$(pwd)/data/maaz_data"
mkdir -p "$OUTPUT_BASE"

# 2) how many episodes you want
NUM=96

for ((i=1; i<=NUM; i++)); do
  echo
  echo "======================================="
  echo "   Starting episode $i of $NUM"
  echo "======================================="

  # Launch exactly one episode
  python3 src/mycobot_stacking_project/scripts/run_episode_collection.py \
    --output-dir "$OUTPUT_BASE" \
    --num-episodes 1 \
    --episode-timeout 120

  echo "→ Episode $i complete, data in $OUTPUT_BASE"
done

echo
echo "All $NUM episodes finished!"
