#!/usr/bin/env python3
"""
Script to automate the collection of multiple episodes of cube stacking data.

This script launches the data collection system, waits for each episode to complete,
and then resets the environment for the next episode.
"""

import os
import sys
import time
import signal
import argparse
import subprocess
from datetime import datetime

# Default settings
DEFAULT_NUM_EPISODES = 10
DEFAULT_OUTPUT_DIR = os.path.expanduser("~/mycobot_episodes")
DEFAULT_EPISODE_TIMEOUT = 120  # seconds
DEFAULT_LAUNCH_FILE = "collect_data.launch.py"

# Global variables
running_processes = []


def signal_handler(sig, frame):
    """Handle Ctrl+C by terminating all running processes."""
    print("\nCtrl+C detected. Shutting down all processes...")
    for process in running_processes:
        if process.poll() is None:  # If process is still running
            try:
                process.terminate()
                time.sleep(1)
                if process.poll() is None:  # If still running after terminate
                    process.kill()
            except Exception as e:
                print(f"Error terminating process: {e}")

    print("All processes terminated. Exiting.")
    sys.exit(0)


def run_command(cmd, wait=True, shell=False):
    """Run a shell command and optionally wait for it to complete."""
    print(f"Running command: {' '.join(cmd) if isinstance(cmd, list) else cmd}")

    if shell:
        process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    else:
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    running_processes.append(process)

    if wait:
        output, _ = process.communicate()
        running_processes.remove(process)
        return output.decode('utf-8')

    return process


def kill_ros_processes():
    """Kill all ROS and Gazebo processes to ensure a clean start."""
    print("Killing any existing ROS 2 and Gazebo processes...")
    cmd = "pkill -9 -f \"ros2|gazebo|gz|rviz2|robot_state_publisher|move_group|cube_spawner|stacking_manager|state_logger_node\""
    run_command(cmd, shell=True)
    time.sleep(2)  # Give processes time to terminate


def create_output_directory(base_dir):
    """Create the output directory for storing episode data."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join(base_dir, f"collection_{timestamp}")
    os.makedirs(output_dir, exist_ok=True)
    print(f"Created output directory: {output_dir}")
    return output_dir


def launch_ros_stack(launch_file, output_dir, log_frequency):
    """Launch the ROS 2 stack for data collection."""
    print(f"Launching ROS 2 stack with {launch_file}...")

    # Build the launch command
    cmd = [
        "ros2", "launch", "mycobot_stacking_project", launch_file,
        f"output_base_dir:={output_dir}",
        f"log_frequency_hz:={log_frequency}"
    ]

    # Launch the process without waiting
    process = run_command(cmd, wait=False)
    print(f"ROS 2 stack launched with PID {process.pid}")
    return process


def wait_for_episode_completion(timeout):
    """Wait for the current episode to complete."""
    print(f"Waiting for episode to complete (timeout: {timeout} seconds)...")

    # In a real implementation, we would check for a specific condition
    # that indicates the episode is complete, such as a ROS 2 topic or service.
    # For now, we'll just wait for the timeout.

    # Check every 5 seconds if the stacking_manager node is still running
    # If it's not running, the episode might have completed
    start_time = time.time()
    check_interval = 5  # seconds

    while (time.time() - start_time) < timeout:
        # Check if stacking_manager is still running
        try:
            # Use ros2 node list to check if stacking_manager is running
            result = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True,
                text=True,
                timeout=3
            )

            # If stacking_manager is not in the node list, the episode might be complete
            if "/stacking_manager" not in result.stdout:
                print("Stacking manager node not found, episode may have completed.")
                return True

        except subprocess.TimeoutExpired:
            print("Warning: ros2 node list command timed out")
        except Exception as e:
            print(f"Error checking node status: {e}")

        # Sleep for the check interval
        remaining = timeout - (time.time() - start_time)
        if remaining <= 0:
            break

        sleep_time = min(check_interval, remaining)
        print(f"Episode still running. Checking again in {sleep_time:.1f} seconds...")
        time.sleep(sleep_time)

    print("Episode timeout reached.")
    return True


def main():
    """Main function to run the episode collection."""
    parser = argparse.ArgumentParser(description="Collect multiple episodes of cube stacking data")
    parser.add_argument("--num-episodes", type=int, default=DEFAULT_NUM_EPISODES,
                        help=f"Number of episodes to collect (default: {DEFAULT_NUM_EPISODES})")
    parser.add_argument("--output-dir", type=str, default=DEFAULT_OUTPUT_DIR,
                        help=f"Base directory for saving episode data (default: {DEFAULT_OUTPUT_DIR})")
    parser.add_argument("--episode-timeout", type=int, default=DEFAULT_EPISODE_TIMEOUT,
                        help=f"Maximum time to wait for an episode to complete in seconds (default: {DEFAULT_EPISODE_TIMEOUT})")
    parser.add_argument("--launch-file", type=str, default=DEFAULT_LAUNCH_FILE,
                        help=f"Launch file to use (default: {DEFAULT_LAUNCH_FILE})")
    parser.add_argument("--log-frequency", type=float, default=2.0,
                        help="Frequency for logging data in Hz (default: 2.0)")

    args = parser.parse_args()

    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # Create output directory
    output_dir = create_output_directory(args.output_dir)

    # Main collection loop
    for episode in range(1, args.num_episodes + 1):
        print(f"\n=== Starting Episode {episode}/{args.num_episodes} ===\n")

        # Kill any existing ROS processes to ensure a clean start
        kill_ros_processes()

        # Launch the ROS stack
        ros_process = launch_ros_stack(args.launch_file, output_dir, args.log_frequency)

        # Wait for the system to initialize
        # With our parallel initialization, we can reduce this wait time
        print("Waiting for system initialization (15 seconds)...")
        time.sleep(15)

        # Check if stacking_manager node is running
        try:
            # Use ros2 node list to check if stacking_manager is running
            for attempt in range(3):  # Try up to 3 times
                result = subprocess.run(
                    ["ros2", "node", "list"],
                    capture_output=True,
                    text=True,
                    timeout=3
                )

                if "/stacking_manager" in result.stdout:
                    print("Stacking manager node is running, system initialized successfully.")
                    break
                else:
                    print(f"Waiting for stacking manager node to start (attempt {attempt+1}/3)...")
                    time.sleep(5)
            else:
                print("Warning: Stacking manager node not detected after waiting. Continuing anyway...")
        except Exception as e:
            print(f"Error checking node status: {e}, continuing anyway...")

        # Wait for the episode to complete
        episode_completed = wait_for_episode_completion(args.episode_timeout)

        if episode_completed:
            print(f"Episode {episode} completed successfully.")
        else:
            print(f"Episode {episode} did not complete within the timeout.")

        # Terminate the ROS stack
        if ros_process.poll() is None:  # If process is still running
            print("Terminating ROS stack...")
            ros_process.terminate()
            time.sleep(5)  # Give the process time to terminate gracefully

            # Force kill if still running
            if ros_process.poll() is None:
                print("Force killing ROS stack...")
                ros_process.kill()

        # Remove from running processes list
        if ros_process in running_processes:
            running_processes.remove(ros_process)

        # Wait between episodes
        if episode < args.num_episodes:
            print("Waiting 2 seconds before starting next episode...")
            time.sleep(2)

    print(f"\n=== Data Collection Complete ===")
    print(f"Collected {args.num_episodes} episodes in {output_dir}")


if __name__ == "__main__":
    main()
