#!/bin/bash
# Script to run diffusion policy inference visualization in Gazebo
# Author: Yukun Wang

# Check if diffusion_policy virtual environment is activated
if [[ "$VIRTUAL_ENV" != *"diffusion_policy"* ]]; then
    echo "ERROR: The diffusion_policy virtual environment is not activated."
    echo "Please activate it first with: source ~/.venvs/diffusion_policy/bin/activate"
    exit 1
fi

echo "Diffusion policy virtual environment is active: $VIRTUAL_ENV"

# Default values
CHECKPOINT_PATH="/home/wangyukun/ros2_ws/DP/checkpoints/model_best-3.pth"
DATA_DIR="/home/wangyukun/mycobot_episodes_test"
IMAGE_SIZE=224
FRAME_SKIP=2
DIFFUSION_TIMESTEPS=200
COMMAND_DELAY=0.5

# Function to display script usage
show_usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -h, --help                   Show this help message"
    echo "  -c, --checkpoint PATH        Path to the model checkpoint (default: $CHECKPOINT_PATH)"
    echo "  -d, --data-dir PATH          Path to the episodes directory (default: $DATA_DIR)"
    echo "  -s, --image-size SIZE        Image size for model input (default: $IMAGE_SIZE)"
    echo "  -f, --frame-skip N           Process every Nth frame (default: $FRAME_SKIP)"
    echo "  -t, --timesteps N            Number of diffusion timesteps (default: $DIFFUSION_TIMESTEPS)"
    echo "  -y, --delay SECONDS          Delay between commands in seconds (default: $COMMAND_DELAY)"
    echo ""
    echo "Example: $0 --frame-skip 3 --timesteps 100"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_usage
            exit 0
            ;;
        -c|--checkpoint)
            CHECKPOINT_PATH="$2"
            shift 2
            ;;
        -d|--data-dir)
            DATA_DIR="$2"
            shift 2
            ;;
        -s|--image-size)
            IMAGE_SIZE="$2"
            shift 2
            ;;
        -f|--frame-skip)
            FRAME_SKIP="$2"
            shift 2
            ;;
        -t|--timesteps)
            DIFFUSION_TIMESTEPS="$2"
            shift 2
            ;;
        -y|--delay)
            COMMAND_DELAY="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Function to clean up existing processes
cleanup() {
    echo "Cleaning up existing ROS 2, Gazebo, and related processes..."
    pkill -9 -f "ros2|gazebo|gz|rviz2|robot_state_publisher|move_group|cube_spawner|stacking_manager"
    sleep 2
    echo "Cleanup complete."
}

# Function to handle Ctrl+C
handle_interrupt() {
    echo "Interrupt received. Cleaning up..."
    cleanup
    exit 0
}

# Set up cleanup trap for Ctrl+C
trap handle_interrupt SIGINT SIGTERM

# Clean up existing processes
cleanup

# Launch Gazebo with the cube stacking world
echo "Starting Gazebo with the cube stacking world..."
ros2 launch mycobot_stacking_project mycobot_gazebo_with_friction.launch.py \
  world_file:=$(ros2 pkg prefix mycobot_stacking_project)/share/mycobot_stacking_project/worlds/cube_stacking_world.world &

# Store the PID of the launch process
LAUNCH_PID=$!

# Set the camera position to match data collection
echo "Setting camera position (after 5 seconds)..."
(
    sleep 5
    gz service -s /gui/move_to/pose \
        --reqtype gz.msgs.GUICamera \
        --reptype gz.msgs.Boolean \
        --timeout 2000 \
        --req "pose: {position: {x: 0.56, y: 0.46, z: 0.36} orientation: {x: 0.02, y: 0.01, z: -0.93, w: 0.36}}"
) &

# Wait for Gazebo to fully initialize
echo "Waiting for Gazebo to initialize (10 seconds)..."
sleep 10

# Run the inference with the model
echo "Starting diffusion policy inference..."
echo "Checkpoint: $CHECKPOINT_PATH"
echo "Data directory: $DATA_DIR"
echo "Frame skip: $FRAME_SKIP"
echo "Diffusion timesteps: $DIFFUSION_TIMESTEPS"
echo "Command delay: $COMMAND_DELAY seconds"

python ~/ros2_ws/DP/inference_gazebo.py \
  --checkpoint_path "$CHECKPOINT_PATH" \
  --data_dir "$DATA_DIR" \
  --image_size "$IMAGE_SIZE" \
  --visualize_trajectory \
  --control_robot \
  --frame_skip "$FRAME_SKIP" \
  --diffusion_timesteps "$DIFFUSION_TIMESTEPS" \
  --command_delay "$COMMAND_DELAY"

# Wait for user to press Ctrl+C
echo "Inference is running. Press Ctrl+C to stop."
wait $LAUNCH_PID
