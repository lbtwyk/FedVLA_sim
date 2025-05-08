#!/bin/bash
# Cube Stacking Launch Script with Debugging Support
# Author: Yukun Wang

# Function to display script usage
show_usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -h, --help                 Show this help message"
    echo "  -d, --debug                Enable debug mode with more verbose output"
    echo "  -c, --clean-only           Only clean up existing processes without launching"
    echo "  -f, --fixed                Use fixed_stacking_task.launch.py instead of start_stacking_task.launch.py"
    echo "  -l, --log-level LEVEL      Set ROS log level (debug, info, warn, error, fatal)"
    echo ""
    echo "Example: $0 --debug --log-level debug"
}

# Default values
DEBUG=false
CLEAN_ONLY=false
USE_FIXED=falseLOG_LEVEL="info"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_usage
            exit 0
            ;;
        -d|--debug)
            DEBUG=true
            shift
            ;;
        -c|--clean-only)
            CLEAN_ONLY=true
            shift
            ;;
        -f|--fixed)
            USE_FIXED=true
            shift
            ;;
        -l|--log-level)
            LOG_LEVEL="$2"
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

# Set up cleanup trap for Ctrl+C
trap cleanup SIGINT SIGTERM

# Always perform cleanup first
cleanup

# Exit if clean-only mode
if [ "$CLEAN_ONLY" = true ]; then
    echo "Clean-only mode selected. Exiting without launching."
    exit 0
fi

# Determine which launch file to use
if [ "$USE_FIXED" = true ]; then
    LAUNCH_FILE="fixed_stacking_task.launch.py"
    echo "Using fixed stacking task launch file."
else
    LAUNCH_FILE="start_stacking_task.launch.py"
    echo "Using standard stacking task launch file."
fi

# Set debug output if enabled
if [ "$DEBUG" = true ]; then
    echo "Debug mode enabled."
    set -x  # Enable command echo
    LOG_LEVEL="debug"  # Override log level to debug
fi

echo "Starting cube stacking simulation with log level: $LOG_LEVEL"
echo "This will launch Gazebo, MoveIt, and the stacking task nodes."
echo "The cube spawner will start after 15 seconds, and the stacking manager after 45 seconds."
echo "Press Ctrl+C to stop all processes."
echo ""

# Launch the stacking task
ros2 launch mycobot_stacking_project $LAUNCH_FILE log_level:=$LOG_LEVEL

# Wait for user to press Ctrl+C
echo "Simulation is running. Press Ctrl+C to stop."
wait
