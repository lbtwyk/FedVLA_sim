#!/usr/bin/env bash
set -e   # abort on any error

# 1) where to put your data
OUTPUT_BASE="$HOME/mycobot_episodes"
mkdir -p "$OUTPUT_BASE"

# 2) how many episodes you want
NUM=50

# Function to kill all ROS processes
kill_ros_processes() {
  echo "Killing any existing ROS processes..."

  # First, kill specific nodes that might have duplicate instances
  echo "→ Killing stacking manager nodes..."
  pkill -9 -f "stacking_manager" || true
  pkill -9 -f "stacking_manager_move_group_interface_context" || true

  echo "→ Killing move_group nodes..."
  pkill -9 -f "move_group" || true

  echo "→ Killing planning scene nodes..."
  pkill -9 -f "planning_scene" || true

  echo "→ Killing state logger nodes..."
  pkill -9 -f "state_logger_node" || true

  echo "→ Killing cube spawner nodes..."
  pkill -9 -f "cube_spawner" || true

  # Then kill all other ROS processes
  echo "→ Killing remaining ROS processes..."
  pkill -9 -f "ros2|gazebo|gz|rviz2|robot_state_publisher" || true

  # Wait a bit longer to ensure all processes are terminated
  sleep 1

  # Double-check for any remaining stacking manager nodes (the most problematic ones)
  if pgrep -f "stacking_manager" > /dev/null; then
    echo "→ WARNING: Stacking manager nodes still running, killing again..."
    pkill -9 -f "stacking_manager" || true
    sleep 1
  fi
}

# Function to check for duplicate nodes
check_duplicate_nodes() {
  echo "→ Checking for duplicate nodes..."

  # Get the list of nodes
  NODE_LIST=$(ros2 node list 2>/dev/null)

  # Check for duplicate stacking_manager nodes
  STACKING_MANAGER_COUNT=$(echo "$NODE_LIST" | grep -c "/stacking_manager")
  if [ "$STACKING_MANAGER_COUNT" -gt 1 ]; then
    echo "→ WARNING: Found $STACKING_MANAGER_COUNT stacking_manager nodes!"
    echo "$NODE_LIST" | grep "/stacking_manager"
    return 1
  fi

  # Check for duplicate move_group nodes
  MOVE_GROUP_COUNT=$(echo "$NODE_LIST" | grep -c "move_group")
  if [ "$MOVE_GROUP_COUNT" -gt 1 ]; then
    echo "→ WARNING: Found $MOVE_GROUP_COUNT move_group nodes!"
    echo "$NODE_LIST" | grep "move_group"
    return 1
  fi

  echo "→ No duplicate nodes found"
  return 0
}

# Function to check robot status
check_robot_status() {
  echo "→ Checking robot status..."

  # Check if stacking_manager node is running
  echo "→ Checking for stacking_manager node:"
  ros2 node list | grep stacking_manager || echo "  No stacking_manager node found!"

  # If stacking_manager is not found, the episode might be complete
  if ! ros2 node list | grep -q "/stacking_manager"; then
    echo "  Stacking manager not found, episode may have completed"
    # We can't set COMPLETED here as it's not in scope
    # Just report the finding
  fi

  # Check for duplicate nodes
  check_duplicate_nodes

  # Check for active topics
  echo "→ Checking for essential topics:"
  ros2 topic list | grep -E 'joint_states|move_group|controller|cube' || echo "  Essential topics not found!"

  # Check for controller status if ros2_control is available
  if ros2 control list_controllers &>/dev/null; then
    echo "→ Controller status:"
    ros2 control list_controllers | grep -v "^$"
  else
    echo "→ ros2_control not available or controllers not running"
  fi

  # Check for recent errors in the logs
  echo "→ Recent errors in logs:"
  ros2 topic echo -n 5 /rosout 2>/dev/null | grep -E 'ERROR|WARN|error|failed|Failed' || echo "  No recent errors found"

  echo "→ Status check complete"
}

# Make sure we clean up on exit
trap kill_ros_processes EXIT

for ((i=1; i<=NUM; i++)); do
  echo
  echo "======================================="
  echo "   Starting episode $i of $NUM"
  echo "======================================="

  # Kill any existing ROS processes before starting
  kill_ros_processes

  # Check for any remaining duplicate nodes
  if check_duplicate_nodes; then
    echo "→ No duplicate nodes found, proceeding with episode"
  else
    echo "→ WARNING: Duplicate nodes detected even after cleanup, attempting to kill again"
    kill_ros_processes
    sleep 2
  fi

  # Launch the episode and monitor its output
  echo "→ Launching episode with verbose output..."

  # Launch directly with ros2 launch
  echo "Launching ROS 2 stack with collect_data.launch.py..."
  echo "→ Collecting data in $OUTPUT_BASE"
  ros2 launch mycobot_stacking_project collect_data.launch.py \
    output_base_dir:=$OUTPUT_BASE \
    log_frequency_hz:=10.0 | tee /tmp/episode_output.log &

  PID=$!

  # Wait for the episode to complete by monitoring the log
  COMPLETED=false
  PLANNING_FAILED=false
  TIMEOUT=65  # Maximum time to wait in seconds
  START_TIME=$(date +%s)
  LAST_ACTIVITY_TIME=$START_TIME
  NO_ACTIVITY_THRESHOLD=20  # seconds to wait before checking status if no activity (reduced from 30)

  while kill -0 $PID 2>/dev/null; do
    # Check if we've exceeded the timeout
    CURRENT_TIME=$(date +%s)
    ELAPSED=$((CURRENT_TIME - START_TIME))
    NO_ACTIVITY_TIME=$((CURRENT_TIME - LAST_ACTIVITY_TIME))

    if [ $ELAPSED -ge $TIMEOUT ]; then
      echo "→ Timeout reached ($TIMEOUT seconds), terminating episode"
      # Kill all ROS processes
      kill_ros_processes
      break
    fi

    # Check if there's been no activity for a while
    if [ $NO_ACTIVITY_TIME -ge $NO_ACTIVITY_THRESHOLD ]; then
      echo "→ No activity detected for ${NO_ACTIVITY_TIME}s, checking robot status..."
      check_robot_status

      # Check if stacking_manager or state_logger_node is running
      if ! ros2 node list | grep -q "/stacking_manager"; then
        echo "→ Stacking manager node not found, episode may have completed"
        COMPLETED=true
        # Kill all ROS processes
        kill_ros_processes
        break
      fi

      # Also check if state_logger_node is running, as it's essential for data collection
      if ! ros2 node list | grep -q "/state_logger_node"; then
        echo "→ State logger node not found, episode may have completed or failed"
        COMPLETED=true
        # Kill all ROS processes
        kill_ros_processes
        break
      fi

      LAST_ACTIVITY_TIME=$CURRENT_TIME  # Reset to avoid repeated checks
    fi

    # Check for absolute failure messages that indicate the task cannot continue
    if grep -q "All retries failed" /tmp/episode_output.log || \
       grep -q "Maximum retries exceeded" /tmp/episode_output.log || \
       grep -q "Failed to execute" /tmp/episode_output.log || \
       grep -q "Failed to move" /tmp/episode_output.log || \
       grep -q "Failed to grasp" /tmp/episode_output.log || \
       grep -q "Failed to place" /tmp/episode_output.log || \
       grep -q "Failed to reach" /tmp/episode_output.log || \
       grep -q "Failed to open gripper" /tmp/episode_output.log || \
       grep -q "Failed to close gripper" /tmp/episode_output.log || \
       grep -q "Failed to detect cubes" /tmp/episode_output.log || \
       grep -q "Failed to attach" /tmp/episode_output.log || \
       grep -q "Failed to detach" /tmp/episode_output.log || \
       grep -q "Failed to compute" /tmp/episode_output.log || \
       grep -q "Failed to control gripper" /tmp/episode_output.log || \
       grep -q "Failed to call data collection service" /tmp/episode_output.log || \
       grep -q "Aborting task" /tmp/episode_output.log || \
       grep -q "Detailed error:" /tmp/episode_output.log || \
       grep -q "Timeout waiting for cubes" /tmp/episode_output.log || \
       grep -q "Controllers not ready" /tmp/episode_output.log; then
      echo "→ Detected stage failure or critical error, aborting episode"
      PLANNING_FAILED=true
      # Kill all ROS processes
      kill_ros_processes
      break
    fi

    # Check for completion messages
    if grep -q "Cube stacking task completed successfully" /tmp/episode_output.log || \
       grep -q "Episode 1 completed successfully" /tmp/episode_output.log || \
       grep -q "Episode .* completed successfully" /tmp/episode_output.log || \
       grep -q "Task completed successfully" /tmp/episode_output.log || \
       grep -q "Successfully placed cube" /tmp/episode_output.log || \
       grep -q "Stacking sequence completed" /tmp/episode_output.log || \
       grep -q "Data Collection Complete" /tmp/episode_output.log || \
       grep -q "Cube stacking completed" /tmp/episode_output.log || \
       grep -q "Stacking task finished" /tmp/episode_output.log || \
       grep -q "Stopping data collection" /tmp/episode_output.log; then
      echo "→ Detected completion message, terminating episode"
      COMPLETED=true
      # Kill all ROS processes
      kill_ros_processes
      break
    fi

    # Check for activity in the logs
    LOG_SIZE_BEFORE=$(wc -l < /tmp/episode_output.log)
    sleep 5
    LOG_SIZE_AFTER=$(wc -l < /tmp/episode_output.log)

    # If log size has changed, there's activity
    if [ "$LOG_SIZE_BEFORE" -ne "$LOG_SIZE_AFTER" ]; then
      LAST_ACTIVITY_TIME=$CURRENT_TIME
    fi

    # Print a status update every 15 seconds
    if [ $((ELAPSED % 15)) -eq 0 ] && [ $ELAPSED -gt 0 ]; then
      echo "→ Episode running for ${ELAPSED}s..."
    fi
  done

  # Make sure all processes are terminated
  echo "→ Ensuring all processes are terminated..."
  kill_ros_processes

  # Report completion status
  if [ "$PLANNING_FAILED" = true ]; then
    echo "→ Episode $i aborted due to planning or stage failure"

    # Instead of deleting files, create a marker file to indicate this episode failed
    # This way, you can manually review and delete if needed
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    FAILURE_MARKER="$OUTPUT_BASE/FAILED_EPISODE_${TIMESTAMP}.txt"

    echo "→ Creating failure marker file: $FAILURE_MARKER"
    echo "Episode $i failed at $(date)" > "$FAILURE_MARKER"
    echo "Reason: Planning or stage failure" >> "$FAILURE_MARKER"
    echo "You may want to manually review and delete the data files created around this time." >> "$FAILURE_MARKER"

    # List recent files for reference but DO NOT delete them
    LATEST_FILES=$(find "$OUTPUT_BASE" -type f \( -name "*.json" -o -name "*.jpg" -o -name "*.png" \) -mmin -5)
    if [ -n "$LATEST_FILES" ]; then
      echo "→ Recent files that may be from the failed episode (NOT deleting):"
      echo "$LATEST_FILES" | head -n 5 | while read file; do
        echo "   $file"
      done
      echo "   ... and more files"
      echo "   These files are preserved for your review."
    else
      echo "→ No recent data files found from this episode"
    fi
  else
    echo "→ Episode $i completed successfully"
  fi

  # Short pause between episodes
  echo "→ Waiting 2 seconds before starting next episode..."
  sleep 2
done

echo
echo "All $NUM episodes finished!"
echo "Data collected in $OUTPUT_BASE"
