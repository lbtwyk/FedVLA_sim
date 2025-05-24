#!/usr/bin/env bash
set -e   # abort on any error

# =====================================================================
# MYCOBOT EPISODE COLLECTION SCRIPT
# =====================================================================
# This script automates the collection of multiple episodes for the
# MyCobot cube stacking task. It handles launching the simulation,
# monitoring the progress, detecting failures, and managing the data.
#
# FAILURE DETECTION MECHANISM:
# ----------------------------
# The script uses a sophisticated failure detection system that:
#
# 1. CATEGORIZES FAILURES:
#    - Planning and execution failures (e.g., "Failed to plan", "All retries failed")
#    - Gripper and manipulation failures (e.g., "Failed to grasp", "Failed to place")
#    - Perception failures (e.g., "Failed to detect cubes", "Timeout waiting for cubes")
#    - System and service failures (e.g., "Controllers not ready", "Failed to call service")
#
# 2. DISTINGUISHES CRITICAL VS. NON-CRITICAL ERRORS:
#    - Critical errors: Prevent episode completion and trigger abortion
#    - Non-critical warnings: Logged but allow the episode to continue
#
# 3. CREATES DETAILED FAILURE REPORTS:
#    - Generates a timestamped FAILED_EPISODE_*.txt file with:
#      * Episode metadata (number, timestamp, duration)
#      * Categorized error messages extracted from logs
#      * List of related data files
#      * Recommendations for handling the failed episode
#
# 4. PRESERVES DATA FOR REVIEW:
#    - Does not automatically delete data from failed episodes
#    - Allows manual review and cleanup using the check_episodes.py script
#
# USAGE:
# ------
# 1. Adjust the parameters at the top of the script (OUTPUT_BASE, NUM, cube positions)
# 2. Run the script: ./collect_multiple_episodes.sh
# 3. Use check_episodes.py to review and clean up failed episodes:
#    - python3 check_episodes.py scan  # to view episode status
#    - python3 check_episodes.py clean # to delete failed episodes
#
# =====================================================================

# 1) where to put your data
OUTPUT_BASE="$HOME/ros2_ws/mycobot_episodes_degrees"
mkdir -p "$OUTPUT_BASE"

# 2) how many episodes you want
NUM=300

# 3) Cube randomization parameters
# Based on cube_stacking_world.world positions ±0.005:
# Yellow cube world position: (0, 0.20) -> bounds: X=[-0.005, 0.005], Y=[0.195, 0.205]
# Orange cube world position: (0.035, 0.25) -> bounds: X=[0.030, 0.040], Y=[0.245, 0.255]

# Yellow cube parameters
YELLOW_MIN_X=-0.005
YELLOW_MAX_X=0.005
YELLOW_MIN_Y=0.195
YELLOW_MAX_Y=0.205

# Orange cube parameters
ORANGE_MIN_X=0.030
ORANGE_MAX_X=0.040
ORANGE_MIN_Y=0.245
ORANGE_MAX_Y=0.255

# Common parameters
MIN_DISTANCE=0.02
RANDOMIZE_POSITIONS=true

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
  echo "→ Yellow cube position bounds: X=[$YELLOW_MIN_X, $YELLOW_MAX_X], Y=[$YELLOW_MIN_Y, $YELLOW_MAX_Y]"
  echo "→ Orange cube position bounds: X=[$ORANGE_MIN_X, $ORANGE_MAX_X], Y=[$ORANGE_MIN_Y, $ORANGE_MAX_Y]"
  echo "→ Minimum distance between cubes: $MIN_DISTANCE"
  echo "→ Randomize positions: $RANDOMIZE_POSITIONS"

  # Create a temporary directory for the generated world file
  TEMP_DIR="/tmp/mycobot_worlds"
  mkdir -p "$TEMP_DIR"

  # Template world file path
  TEMPLATE_WORLD_PATH="$(ros2 pkg prefix mycobot_stacking_project)/share/mycobot_stacking_project/worlds/cube_stacking_world.world"

  # Output world file path - use a unique name for each episode to avoid conflicts
  OUTPUT_WORLD_PATH="$TEMP_DIR/cube_stacking_world_randomized_${i}.world"

  echo "→ Template world file: $TEMPLATE_WORLD_PATH"
  echo "→ Output world file: $OUTPUT_WORLD_PATH"

  ros2 launch mycobot_stacking_project collect_data.launch.py \
    output_base_dir:=$OUTPUT_BASE \
    log_frequency_hz:=10.0 \
    yellow_min_x:=$YELLOW_MIN_X \
    yellow_max_x:=$YELLOW_MAX_X \
    yellow_min_y:=$YELLOW_MIN_Y \
    yellow_max_y:=$YELLOW_MAX_Y \
    orange_min_x:=$ORANGE_MIN_X \
    orange_max_x:=$ORANGE_MAX_X \
    orange_min_y:=$ORANGE_MIN_Y \
    orange_max_y:=$ORANGE_MAX_Y \
    min_distance_between_cubes:=$MIN_DISTANCE \
    randomize_positions:=$RANDOMIZE_POSITIONS \
    template_world_path:=$TEMPLATE_WORLD_PATH \
    output_world_path:=$OUTPUT_WORLD_PATH | tee /tmp/episode_output.log &

  PID=$!

  # Wait for the episode to complete by monitoring the log
  COMPLETED=false
  PLANNING_FAILED=false
  TIMEOUT=50  # Maximum time to wait in seconds
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

# =====================================================================
# FAILURE DETECTION LOGIC
# =====================================================================
# This section contains patterns for detecting critical failures that prevent
# episode completion. We categorize failures into different types and only
# abort the episode for truly critical errors that cannot be recovered from.
# =====================================================================

    # Function to check if a critical error pattern is found in the log
    check_critical_error() {
      local pattern="$1"
      local description="$2"

      if grep -q "$pattern" /tmp/episode_output.log; then
        echo "→ CRITICAL ERROR: $description"
        return 0  # Error found
      fi
      return 1  # No error found
    }

    # Check for critical errors that should abort the episode
    CRITICAL_ERROR_FOUND=false

    # Category 1: Planning and execution failures
    # These are errors that prevent the robot from completing the task
    if check_critical_error "All retries failed" "All planning retries failed" || \
       check_critical_error "Maximum retries exceeded" "Maximum planning retries exceeded" || \
       check_critical_error "Failed to plan .* after [0-9]+ attempts" "Planning failed after multiple attempts" || \
       check_critical_error "Detailed error: Planning failed" "MoveIt planning failed" || \
       check_critical_error "Detailed error: Invalid motion plan" "Invalid motion plan generated" || \
       check_critical_error "Detailed error: Control failed" "Robot control failed" || \
       check_critical_error "Aborting task" "Task explicitly aborted"; then
      CRITICAL_ERROR_FOUND=true
    fi

    # Category 2: Gripper and manipulation failures
    # These are errors that prevent the robot from manipulating objects
    if check_critical_error "Failed to grasp .* after [0-9]+ attempts" "Failed to grasp object after multiple attempts" || \
       check_critical_error "Failed to place .* after [0-9]+ attempts" "Failed to place object after multiple attempts" || \
       check_critical_error "Failed to control gripper" "Critical gripper control failure" || \
       check_critical_error "Failed to attach .* to gripper" "Failed to attach object to gripper" || \
       check_critical_error "Failed to detach .* from gripper" "Failed to detach object from gripper"; then
      CRITICAL_ERROR_FOUND=true
    fi

    # Category 3: Perception failures
    # These are errors that prevent the robot from perceiving the environment
    if check_critical_error "Failed to detect cubes after [0-9]+ attempts" "Failed to detect cubes after multiple attempts" || \
       check_critical_error "Timeout waiting for cubes" "Timeout waiting for cubes to be detected"; then
      CRITICAL_ERROR_FOUND=true
    fi

    # Category 4: System and service failures
    # These are errors that indicate system-level issues
    if check_critical_error "Controllers not ready after [0-9]+ seconds" "Controllers failed to initialize" || \
       check_critical_error "Failed to call data collection service" "Data collection service failure" || \
       check_critical_error "InitStageException caught" "Task initialization failed"; then
      CRITICAL_ERROR_FOUND=true
    fi

    # If any critical error was found, abort the episode
    if [ "$CRITICAL_ERROR_FOUND" = true ]; then
      echo "→ Detected critical error that prevents episode completion, aborting episode"
      PLANNING_FAILED=true
      # Kill all ROS processes
      kill_ros_processes
      break
    fi

    # Check for non-critical warnings that should NOT abort the episode
    # These patterns are explicitly ignored to prevent false positives
    NON_CRITICAL_PATTERNS=(
      "Failed to plan .* but continuing due to expected collision"  # Expected collision, can continue
      "Warning: .* not detected"  # Warning only, not critical
      "WARN: .* parameter not set"  # Parameter warning, not critical
      "WARN: .* deprecated"  # Deprecation warning, not critical
      "WARN: .* slower than expected"  # Performance warning, not critical
    )

    # Log non-critical warnings for information only
    for pattern in "${NON_CRITICAL_PATTERNS[@]}"; do
      if grep -q "$pattern" /tmp/episode_output.log; then
        echo "→ Non-critical warning detected: $pattern (continuing episode)"
      fi
    done

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
    echo "→ Episode $i aborted due to critical failure"

    # Create a detailed marker file to indicate this episode failed
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    FAILURE_MARKER="$OUTPUT_BASE/FAILED_EPISODE_${TIMESTAMP}.txt"

    echo "→ Creating detailed failure marker file: $FAILURE_MARKER"

    # Write header information to the failure marker file
    echo "===============================================" > "$FAILURE_MARKER"
    echo "EPISODE FAILURE REPORT" >> "$FAILURE_MARKER"
    echo "===============================================" >> "$FAILURE_MARKER"
    echo "Episode: $i" >> "$FAILURE_MARKER"
    echo "Timestamp: $(date)" >> "$FAILURE_MARKER"
    echo "Duration: $ELAPSED seconds" >> "$FAILURE_MARKER"
    echo "" >> "$FAILURE_MARKER"

    # Extract and include critical error messages from the log
    echo "CRITICAL ERROR MESSAGES:" >> "$FAILURE_MARKER"
    echo "-----------------------------------------------" >> "$FAILURE_MARKER"

    # Extract error messages for each category
    echo "Planning and execution errors:" >> "$FAILURE_MARKER"
    grep -E "All retries failed|Maximum retries exceeded|Failed to plan|Detailed error:|Aborting task" /tmp/episode_output.log | tail -n 10 >> "$FAILURE_MARKER" || echo "None found" >> "$FAILURE_MARKER"

    echo "" >> "$FAILURE_MARKER"
    echo "Gripper and manipulation errors:" >> "$FAILURE_MARKER"
    grep -E "Failed to grasp|Failed to place|Failed to control gripper|Failed to attach|Failed to detach" /tmp/episode_output.log | tail -n 10 >> "$FAILURE_MARKER" || echo "None found" >> "$FAILURE_MARKER"

    echo "" >> "$FAILURE_MARKER"
    echo "Perception errors:" >> "$FAILURE_MARKER"
    grep -E "Failed to detect cubes|Timeout waiting for cubes" /tmp/episode_output.log | tail -n 10 >> "$FAILURE_MARKER" || echo "None found" >> "$FAILURE_MARKER"

    echo "" >> "$FAILURE_MARKER"
    echo "System and service errors:" >> "$FAILURE_MARKER"
    grep -E "Controllers not ready|Failed to call data collection service|InitStageException" /tmp/episode_output.log | tail -n 10 >> "$FAILURE_MARKER" || echo "None found" >> "$FAILURE_MARKER"

    echo "" >> "$FAILURE_MARKER"
    echo "-----------------------------------------------" >> "$FAILURE_MARKER"
    echo "RECOMMENDATION: This episode contains critical errors that prevented successful completion." >> "$FAILURE_MARKER"
    echo "You may want to review and delete the data files created during this episode." >> "$FAILURE_MARKER"
    echo "===============================================" >> "$FAILURE_MARKER"

    # List recent files for reference but DO NOT delete them
    LATEST_FILES=$(find "$OUTPUT_BASE" -type f \( -name "*.json" -o -name "*.jpg" -o -name "*.png" \) -mmin -5)
    if [ -n "$LATEST_FILES" ]; then
      echo "→ Recent files that may be from the failed episode (NOT deleting):"
      echo "$LATEST_FILES" | head -n 5 | while read file; do
        echo "   $file"
      done

      # Add file list to the failure marker
      echo "" >> "$FAILURE_MARKER"
      echo "RELATED FILES:" >> "$FAILURE_MARKER"
      echo "$LATEST_FILES" | head -n 20 >> "$FAILURE_MARKER"

      echo "   ... and more files"
      echo "   These files are preserved for your review."
    else
      echo "→ No recent data files found from this episode"
      echo "No recent data files found" >> "$FAILURE_MARKER"
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
