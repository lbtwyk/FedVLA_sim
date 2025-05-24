#!/bin/bash
# Script to set up a virtual environment with the necessary dependencies

# Default values
VENV_DIR="$HOME/.venvs/diffusion_policy"
PYTHON_CMD="python3"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --venv-dir)
      VENV_DIR="$2"
      shift 2
      ;;
    --python)
      PYTHON_CMD="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

# Check if Python is available
if ! command -v $PYTHON_CMD &> /dev/null; then
  echo "Error: $PYTHON_CMD not found"
  exit 1
fi

# Create the virtual environment directory if it doesn't exist
mkdir -p $(dirname "$VENV_DIR")

# Create the virtual environment
echo "Creating virtual environment at $VENV_DIR..."
$PYTHON_CMD -m venv "$VENV_DIR"

# Activate the virtual environment
source "$VENV_DIR/bin/activate"

# Install dependencies
echo "Installing dependencies..."
pip install --upgrade pip
pip install torch torchvision pillow opencv-python

# Create a wrapper script to run the simulation with the virtual environment
WRAPPER_SCRIPT="$HOME/ros2_ws/run_inference_with_venv.sh"
cat > "$WRAPPER_SCRIPT" << EOF
#!/bin/bash
# Wrapper script to run simulation inference with the virtual environment

# Activate the virtual environment
source "$VENV_DIR/bin/activate"

# Source ROS 2 workspace
source "$HOME/ros2_ws/install/setup.bash"

# Run the simulation inference script
"$HOME/ros2_ws/src/diffusion_policy_inference/run_simulation_inference.sh" "\$@"
EOF

# Make the wrapper script executable
chmod +x "$WRAPPER_SCRIPT"

echo "Setup complete!"
echo "To run the simulation inference with the virtual environment, use:"
echo "$WRAPPER_SCRIPT --checkpoint /path/to/your/model_best.pth --model-dir /path/to/your/model/directory"
