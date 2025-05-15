#!/usr/bin/env bash
set -e

# 1) Target location for episodes
OUTPUT_BASE="$(pwd)/src/mycobot_episodes"
mkdir -p "$OUTPUT_BASE"

# 2) Number of episodes to collect (default to 96 if not given)
NUM=${1:-96}

for ((i=1; i<=NUM; i++)); do
  echo
  echo "======================================="
  echo "   Starting episode $i of $NUM"
  echo "======================================="

  python3 src/mycobot_stacking_project/scripts/run_episode_collection.py \
    --output-dir "$OUTPUT_BASE" \
    --num-episodes 1 \
    --episode-timeout 120

  echo "→ Episode $i complete, data in $OUTPUT_BASE"
done

echo
echo "All $NUM episodes finished!"

