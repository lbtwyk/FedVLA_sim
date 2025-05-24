#!/bin/bash
# Script to run simulation inference with the diffusion policy model

# Default values
CHECKPOINT_PATH=""
MODEL_DIR=""
INFERENCE_RATE=10.0
LOG_LEVEL="info"

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
    --rate)
      INFERENCE_RATE="$2"
      shift 2
      ;;
    --log-level)
      LOG_LEVEL="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

# Check if checkpoint path is provided
if [ -z "$CHECKPOINT_PATH" ]; then
  echo "Error: Checkpoint path is required"
  echo "Usage: $0 --checkpoint PATH [--model-dir PATH] [--rate RATE] [--log-level LEVEL]"
  exit 1
fi

# Check if checkpoint file exists
if [ ! -f "$CHECKPOINT_PATH" ]; then
  echo "Error: Checkpoint file not found: $CHECKPOINT_PATH"
  exit 1
fi

# Clean up any existing ROS processes
echo "Cleaning up existing ROS processes..."
pkill -9 -f "ros2|gazebo|gz|rviz2|robot_state_publisher|move_group|cube_spawner|model_inference_node"
sleep 2

# Launch the simulation with inference
echo "Starting simulation inference..."
echo "Checkpoint: $CHECKPOINT_PATH"
echo "Model directory: $MODEL_DIR"
echo "Inference rate: $INFERENCE_RATE Hz"
echo "Log level: $LOG_LEVEL"

# Launch the simulation
ros2 launch diffusion_policy_inference simulation_inference.launch.py \
  checkpoint_path:="$CHECKPOINT_PATH" \
  model_dir:="$MODEL_DIR" \
  inference_rate:="$INFERENCE_RATE" \
  log_level:="$LOG_LEVEL"

# Wait for user to press Ctrl+C
echo "Simulation is running. Press Ctrl+C to stop."
wait
