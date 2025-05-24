#!/bin/bash
# Script to run real-time inference with the diffusion policy model

# Global variable to track background processes
GAZEBO_PID=""
INFERENCE_PID=""

# Function to handle cleanup on exit
cleanup() {
    echo ""
    echo "Received interrupt signal. Cleaning up..."

    # Kill the inference process if it's running
    if [ -n "$INFERENCE_PID" ]; then
        echo "Stopping inference process (PID: $INFERENCE_PID)..."
        kill -TERM "$INFERENCE_PID" 2>/dev/null
        sleep 2
        # Force kill if still running
        kill -KILL "$INFERENCE_PID" 2>/dev/null
    fi

    # Kill Gazebo and related processes
    echo "Stopping Gazebo and ROS processes..."
    pkill -9 -f "ros2|gazebo|gz|rviz2|robot_state_publisher|move_group|cube_spawner|model_inference_node" 2>/dev/null

    echo "Cleanup completed."
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Default parameters
CHECKPOINT_PATH="$HOME/ros2_ws/DP/checkpoints/model_epoch_560.pth"  # Using the model path from user's memory
MODEL_DIR="$HOME/ros2_ws/DP"
INFERENCE_RATE=10.0
COMMAND_DELAY=0.1
VISUALIZE=true
CONTROL_ROBOT=true
STARTUP_DELAY=10
LOG_LEVEL="info"
INITIAL_POSITION="home"
INITIAL_HEIGHT=0.3
CUSTOM_JOINTS=""
TIME_EMB_MULT=2  # Using 2 as detected from your trained model (time_mlp.1.weight shape [128, 64])
HIDDEN_DIM=256  # Using 256 as specified in train_original.py (line 808)
IMAGE_FEATURE_DIM=512  # Using 512 as specified for ResNet34
IMAGE_SIZE=300  # Using 300 as specified in train_original.py (line 805)

# Function to display usage information
show_usage() {
  echo "Usage: $0 [options]"
  echo "Options:"
  echo "  --checkpoint PATH   Path to the model checkpoint (default: $CHECKPOINT_PATH)"
  echo "  --model-dir PATH    Path to the directory containing model.py and train.py (default: $MODEL_DIR)"
  echo "  --time-emb-mult N   Time embedding multiplier (default: $TIME_EMB_MULT)"
  echo "  --hidden-dim N      Hidden dimension size (default: $HIDDEN_DIM)"
  echo "  --image-feature-dim N  Image feature dimension (default: $IMAGE_FEATURE_DIM)"
  echo "  --image-size N      Image size for resizing (default: $IMAGE_SIZE)"
  echo "  --rate RATE         Inference rate in Hz (default: $INFERENCE_RATE)"
  echo "  --delay DELAY       Command delay in seconds (default: $COMMAND_DELAY)"
  echo "  --startup SECONDS   Delay after Gazebo starts before running inference (default: $STARTUP_DELAY)"
  echo "  --log-level LEVEL   Set log level: debug, info, warn, error (default: $LOG_LEVEL)"
  echo "  --initial-pos POS   Set initial arm position: home, ready, forward, custom (default: $INITIAL_POSITION)"
  echo "  --initial-height H  Set target height for forward position in meters (default: $INITIAL_HEIGHT)"
  echo "  --custom-joints \"J1 J2 J3 J4 J5 J6\"  Custom joint angles in radians"
  echo "  --no-visualize      Disable visualization"
  echo "  --no-control        Disable robot control"
  echo "  --help              Show this help message"
  exit 0
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --checkpoint)
      CHECKPOINT_PATH="$2"
      shift 2
      ;;
    --model-dir)
      MODEL_DIR="$2"
      shift 2
      ;;

    --time-emb-mult)
      TIME_EMB_MULT="$2"
      # Validate time embedding multiplier is a positive integer
      if ! [[ "$TIME_EMB_MULT" =~ ^[1-9][0-9]*$ ]]; then
        echo "Invalid time embedding multiplier: $TIME_EMB_MULT. Must be a positive integer."
        exit 1
      fi
      shift 2
      ;;
    --hidden-dim)
      HIDDEN_DIM="$2"
      # Validate hidden dimension is a positive integer
      if ! [[ "$HIDDEN_DIM" =~ ^[1-9][0-9]*$ ]]; then
        echo "Invalid hidden dimension: $HIDDEN_DIM. Must be a positive integer."
        exit 1
      fi
      shift 2
      ;;
    --image-feature-dim)
      IMAGE_FEATURE_DIM="$2"
      # Validate image feature dimension is a positive integer
      if ! [[ "$IMAGE_FEATURE_DIM" =~ ^[1-9][0-9]*$ ]]; then
        echo "Invalid image feature dimension: $IMAGE_FEATURE_DIM. Must be a positive integer."
        exit 1
      fi
      shift 2
      ;;
    --image-size)
      IMAGE_SIZE="$2"
      # Validate image size is a positive integer
      if ! [[ "$IMAGE_SIZE" =~ ^[1-9][0-9]*$ ]]; then
        echo "Invalid image size: $IMAGE_SIZE. Must be a positive integer."
        exit 1
      fi
      shift 2
      ;;
    --rate)
      INFERENCE_RATE="$2"
      shift 2
      ;;
    --delay)
      COMMAND_DELAY="$2"
      shift 2
      ;;
    --startup)
      STARTUP_DELAY="$2"
      shift 2
      ;;
    --log-level)
      LOG_LEVEL="$2"
      # Validate log level
      if [[ ! "$LOG_LEVEL" =~ ^(debug|info|warn|error)$ ]]; then
        echo "Invalid log level: $LOG_LEVEL. Must be one of: debug, info, warn, error"
        exit 1
      fi
      shift 2
      ;;
    --initial-pos)
      INITIAL_POSITION="$2"
      # Validate initial position
      if [[ ! "$INITIAL_POSITION" =~ ^(home|ready|forward|custom)$ ]]; then
        echo "Invalid initial position: $INITIAL_POSITION. Must be one of: home, ready, forward, custom"
        exit 1
      fi
      shift 2
      ;;
    --initial-height)
      INITIAL_HEIGHT="$2"
      shift 2
      ;;
    --custom-joints)
      CUSTOM_JOINTS="$2"
      shift 2
      ;;
    --no-visualize)
      VISUALIZE=false
      shift
      ;;
    --no-control)
      CONTROL_ROBOT=false
      shift
      ;;
    --help)
      show_usage
      ;;
    *)
      echo "Unknown option: $1"
      show_usage
      ;;
  esac
done

# Check if checkpoint exists
if [ ! -f "$CHECKPOINT_PATH" ]; then
  echo "Error: Checkpoint file not found: $CHECKPOINT_PATH"
  exit 1
fi

# Clean up any existing ROS processes
echo "Cleaning up existing ROS processes..."
pkill -9 -f "ros2|gazebo|gz|rviz2|robot_state_publisher|move_group|cube_spawner|model_inference_node"
sleep 2

# Launch Gazebo with the cube stacking world
echo "Starting Gazebo with cube stacking world..."
WORLD_PATH=$(ros2 pkg prefix mycobot_stacking_project)/share/mycobot_stacking_project/worlds/cube_stacking_world.world
if [ ! -f "$WORLD_PATH" ]; then
  echo "Error: World file not found: $WORLD_PATH"
  exit 1
fi

ros2 launch mycobot_stacking_project mycobot_gazebo_with_friction.launch.py world_file:=$WORLD_PATH &
GAZEBO_PID=$!

# Wait for Gazebo to fully initialize
echo "Waiting for Gazebo to initialize ($STARTUP_DELAY seconds)..."
sleep $STARTUP_DELAY

# Set the camera position to match data collection
echo "Setting camera position..."
gz service -s /gui/move_to/pose \
    --reqtype gz.msgs.GUICamera \
    --reptype gz.msgs.Boolean \
    --timeout 2000 \
    --req "pose: {position: {x: 0.56, y: 0.46, z: 0.36} orientation: {x: 0.02, y: 0.01, z: -0.93, w: 0.36}}"

# Run the inference with the model
echo "Starting real-time diffusion policy inference..."
echo "Checkpoint: $CHECKPOINT_PATH"
echo "Model Directory: $MODEL_DIR"
echo "ResNet Backbone: ResNet34"
echo "Time Embedding Multiplier: $TIME_EMB_MULT"
echo "Hidden Dimension: $HIDDEN_DIM"
echo "Image Feature Dimension: $IMAGE_FEATURE_DIM"
echo "Image Size: ${IMAGE_SIZE}x${IMAGE_SIZE}"
echo "Inference rate: $INFERENCE_RATE Hz"
echo "Command delay: $COMMAND_DELAY seconds"

# Activate the virtual environment if it exists
if [ -f "$HOME/.venvs/diffusion_policy/bin/activate" ]; then
    echo "Activating virtual environment at $HOME/.venvs/diffusion_policy..."
    source "$HOME/.venvs/diffusion_policy/bin/activate"
    PYTHON_CMD="python"
elif [ -f "$HOME/ros2_ws/DP/venv/bin/activate" ]; then
    echo "Activating virtual environment at $HOME/ros2_ws/DP/venv..."
    source "$HOME/ros2_ws/DP/venv/bin/activate"
    PYTHON_CMD="python"
elif [ -f "$HOME/ros2_ws/DP/.venv/bin/activate" ]; then
    echo "Activating virtual environment at $HOME/ros2_ws/DP/.venv..."
    source "$HOME/ros2_ws/DP/.venv/bin/activate"
    PYTHON_CMD="python"
elif [ -f "$HOME/ros2_ws/venv/bin/activate" ]; then
    echo "Activating virtual environment at $HOME/ros2_ws/venv..."
    source "$HOME/ros2_ws/venv/bin/activate"
    PYTHON_CMD="python"
elif [ -f "$HOME/.venv/bin/activate" ]; then
    echo "Activating virtual environment at $HOME/.venv..."
    source "$HOME/.venv/bin/activate"
    PYTHON_CMD="python"
else
    echo "No virtual environment found, using system Python"
    PYTHON_CMD="python3"
fi

# Build the command with appropriate flags
CMD="$PYTHON_CMD $HOME/ros2_ws/DP/inference_realtime.py --checkpoint_path \"$CHECKPOINT_PATH\" --model_dir \"$MODEL_DIR\" --inference_rate $INFERENCE_RATE --command_delay $COMMAND_DELAY --time_emb_mult $TIME_EMB_MULT --hidden_dim $HIDDEN_DIM --image_feature_dim $IMAGE_FEATURE_DIM --image_size $IMAGE_SIZE"

# Add initial position arguments
CMD="$CMD --initial_position $INITIAL_POSITION --initial_height $INITIAL_HEIGHT"

if [ "$INITIAL_POSITION" = "custom" ] && [ -n "$CUSTOM_JOINTS" ]; then
  CMD="$CMD --custom_joints $CUSTOM_JOINTS"
fi

# Add ROS2 arguments separately - they need to be passed differently
export RCUTILS_LOGGING_LEVEL=$LOG_LEVEL

if [ "$VISUALIZE" = true ]; then
  CMD="$CMD --visualize_trajectory"
  echo "Visualization: Enabled"
else
  echo "Visualization: Disabled"
fi

if [ "$CONTROL_ROBOT" = true ]; then
  CMD="$CMD --control_robot"
  echo "Robot Control: Enabled"
else
  echo "Robot Control: Disabled"
fi

echo "Initial Position: $INITIAL_POSITION"
if [ "$INITIAL_POSITION" = "forward" ]; then
  echo "Initial Height: $INITIAL_HEIGHT meters"
elif [ "$INITIAL_POSITION" = "custom" ]; then
  echo "Custom Joints: $CUSTOM_JOINTS"
fi
echo "Log Level: $LOG_LEVEL"

# Run the command in background
echo "Running: $CMD"
eval $CMD &
INFERENCE_PID=$!

# Wait for user to press Ctrl+C
echo "Real-time inference is running (PID: $INFERENCE_PID). Press Ctrl+C to stop."

# Wait for the inference process to complete or for interrupt signal
while kill -0 "$INFERENCE_PID" 2>/dev/null; do
    sleep 1
done

# If we reach here, the inference process ended naturally
echo "Inference process completed."
cleanup
