#!/usr/bin/env python3
"""
Inference script for running diffusion policy model on recorded episodes
and visualizing the results in Gazebo simulation.
"""

import os
import sys
import json
import glob
import time
import random
import logging
import argparse
import numpy as np
from typing import Optional, List, Dict
from tqdm import tqdm

# PyTorch imports
import torch
import torch.nn.functional as F
from torchvision import transforms
from PIL import Image

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory, GripperCommand
from builtin_interfaces.msg import Duration

# Matplotlib for visualization
import matplotlib.pyplot as plt

# Add the model directory to the path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import model and training functions
from model import DiffusionPolicyModel
from train import linear_beta_schedule, p_sample_loop

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# TQDM bar format
TQDM_BAR_FORMAT = '{l_bar}{bar}| {n_fmt}/{total_fmt} [{elapsed}<{remaining}, {rate_fmt}{postfix}]'





class GazeboInferenceNode(Node):
    """ROS2 node for running inference and controlling the robot in Gazebo."""

    def __init__(self, args):
        """Initialize the node with the provided arguments."""
        super().__init__('gazebo_inference_node')

        # Store arguments
        self.args = args

        # Create callback groups for actions
        self.action_callback_group = ReentrantCallbackGroup()

        # Create action clients
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self.action_callback_group
        )

        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_action_controller/gripper_cmd',
            callback_group=self.action_callback_group
        )

        # Define joint names
        self.arm_joint_names = [
            'link1_to_link2', 'link2_to_link3', 'link3_to_link4',
            'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange'
        ]

        # Wait for action servers
        self.get_logger().info("Waiting for action servers...")
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Arm action server not available")
            rclpy.shutdown()
            sys.exit(1)

        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Gripper action server not available")
            rclpy.shutdown()
            sys.exit(1)

        self.get_logger().info("Action servers connected!")

    def send_arm_command(self, joint_positions):
        """Send a command to the arm."""
        # Convert joint positions from degrees to radians for robot control
        joint_positions_rad = [pos * 3.14159265359 / 180.0 for pos in joint_positions]

        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_positions_rad
        point.time_from_start = Duration(sec=1, nanosec=0)  # 1 second duration

        trajectory.points = [point]

        # Create goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        # Send goal
        self.get_logger().info(f"Sending arm command: {joint_positions}")
        self.arm_client.send_goal_async(goal_msg)

    def send_gripper_command(self, gripper_value):
        """Send a command to the gripper."""
        # Map gripper value (0-100) to position (-0.5 to 0.0)
        # 0 = closed, 100 = open
        position = -0.5 * (1.0 - gripper_value / 100.0)

        # Create goal message
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 5.0  # Use moderate effort

        # Send goal
        self.get_logger().info(f"Sending gripper command: {position:.4f}")
        self.gripper_client.send_goal_async(goal_msg)


def load_episode_data(episode_path: str) -> Optional[List[Dict]]:
    """Loads the states.json data for a specified episode directory."""
    # Look for states.json (plural) instead of state.json
    state_file_path = os.path.join(episode_path, 'states.json')

    if not os.path.isdir(episode_path):
        logging.error(f"Episode directory not found: {episode_path}")
        return None
    if not os.path.isfile(state_file_path):
        logging.error(f"states.json not found in {episode_path}")
        return None

    try:
        with open(state_file_path, 'r') as f:
            episode_data = json.load(f)
        if not isinstance(episode_data, list):
            logging.error(f"states.json in {episode_path} is not a list.")
            return None
        # Add full image path to each step for convenience
        for step_data in episode_data:
             if isinstance(step_data.get("image"), str):
                  step_data["full_image_path"] = os.path.join(episode_path, step_data["image"])
             else:
                  logging.warning(f"Invalid image path found in {state_file_path} for step: {step_data}")
                  step_data["full_image_path"] = None # Mark as invalid

        return episode_data
    except json.JSONDecodeError:
        logging.error(f"Error decoding JSON from {state_file_path}")
        return None
    except Exception as e:
        logging.exception(f"An unexpected error occurred loading episode {episode_path}: {e}")
        return None


def find_episode_directories(data_dir: str) -> List[str]:
    """Find all episode directories in the data directory structure."""
    # Look for directories matching the pattern episode_*
    episode_dirs = []

    # First, look directly in the data_dir
    direct_episodes = glob.glob(os.path.join(data_dir, "episode_*"))
    episode_dirs.extend(direct_episodes)

    # Then look in collection_* subdirectories
    collection_dirs = glob.glob(os.path.join(data_dir, "collection_*"))
    for collection_dir in collection_dirs:
        nested_episodes = glob.glob(os.path.join(collection_dir, "episode_*"))
        episode_dirs.extend(nested_episodes)

    return episode_dirs


def run_inference_in_gazebo(args):
    """
    Loads a trained model and runs inference on an episode,
    sending the predicted states to the robot in Gazebo.
    """
    # --- Initialize ROS2 ---
    rclpy.init(args=None)

    # --- Device Setup ---
    device = torch.device(f"cuda:{args.gpu_id}" if torch.cuda.is_available() else "cpu")
    logging.info(f"Using device: {device}")

    # --- Load Checkpoint ---
    if not os.path.isfile(args.checkpoint_path):
        logging.error(f"Checkpoint file not found: {args.checkpoint_path}")
        rclpy.shutdown()
        return
    logging.info(f"Loading checkpoint from: {args.checkpoint_path}")
    # Explicitly set weights_only=False to handle numpy scalar objects in the checkpoint
    checkpoint = torch.load(args.checkpoint_path, map_location=device, weights_only=False)

    # Load training arguments from checkpoint
    train_args = checkpoint.get('args')
    if train_args is None:
        logging.error("Checkpoint missing training arguments ('args'). Attempting fallback.")
        train_args = vars(args) # Use current args as fallback
    elif isinstance(train_args, argparse.Namespace):
        train_args = vars(train_args)
    logging.info("Loaded training arguments from checkpoint.")

    # --- Diffusion Schedule Setup ---
    timesteps = train_args.get('diffusion_timesteps', args.diffusion_timesteps)
    beta_start = train_args.get('beta_start', args.beta_start)
    beta_end = train_args.get('beta_end', args.beta_end)
    betas = linear_beta_schedule(timesteps=timesteps, beta_start=beta_start, beta_end=beta_end).to(device)
    alphas = 1. - betas
    alphas_cumprod = torch.cumprod(alphas, axis=0)
    alphas_cumprod_prev = F.pad(alphas_cumprod[:-1], (1, 0), value=1.0)
    sqrt_one_minus_alphas_cumprod = torch.sqrt(1. - alphas_cumprod)
    sqrt_recip_alphas = torch.sqrt(1.0 / alphas)
    posterior_variance = betas * (1. - alphas_cumprod_prev) / (1. - alphas_cumprod)
    logging.info(f"Diffusion schedule set up with {timesteps} timesteps.")

    # --- Model Initialization ---
    state_dim = train_args.get('state_dim', args.state_dim) # Get state_dim from loaded args
    model = DiffusionPolicyModel(
        state_dim=state_dim,
        time_emb_dim=train_args.get('time_emb_dim', args.time_emb_dim),
        hidden_dim=train_args.get('hidden_dim', args.hidden_dim),
        num_layers=train_args.get('num_mlp_layers', args.num_mlp_layers),
        image_feature_dim=train_args.get('image_feature_dim', args.image_feature_dim),
        use_pretrained_resnet=train_args.get('use_pretrained_resnet', True),
        freeze_resnet=train_args.get('freeze_resnet', True)
    ).to(device)
    try:
        model.load_state_dict(checkpoint['model_state_dict'])
        logging.info("Successfully loaded model state dict.")
    except Exception as e:
         logging.exception(f"Error loading state dict: {e}")
         rclpy.shutdown()
         return
    model.eval()

    # --- Image Transform ---
    image_size = train_args.get('image_size', args.image_size)
    image_transform = transforms.Compose([
        transforms.Resize((image_size, image_size)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])

    # --- Find Episode Directories ---
    episode_dirs = find_episode_directories(args.data_dir)
    if not episode_dirs:
        logging.error(f"No episode directories found in {args.data_dir}")
        rclpy.shutdown()
        return

    logging.info(f"Found {len(episode_dirs)} episode directories")

    # --- Episode Selection ---
    episode_to_run = None
    if args.episode_path:
        # Use the specified episode path
        if os.path.isdir(args.episode_path):
            episode_to_run = args.episode_path
        else:
            logging.error(f"Specified episode path not found: {args.episode_path}")
            rclpy.shutdown()
            return
    else:
        # Select a random episode
        episode_to_run = random.choice(episode_dirs)

    logging.info(f"Selected episode: {episode_to_run}")

    # --- Load Data for Selected Episode ---
    episode_timesteps = load_episode_data(episode_to_run)
    if episode_timesteps is None or not episode_timesteps:
        logging.error(f"Failed to load or empty data for episode {episode_to_run}.")
        rclpy.shutdown()
        return
    logging.info(f"Loaded {len(episode_timesteps)} timesteps for episode {os.path.basename(episode_to_run)}.")

    # --- Create ROS2 Node ---
    gazebo_node = GazeboInferenceNode(args)
    executor = MultiThreadedExecutor()
    executor.add_node(gazebo_node)

    # --- Visualization Setup ---
    vis_fig, vis_ax = None, None
    if args.visualize_trajectory:
        logging.info("Visualization enabled. Make sure matplotlib window is visible.")
        plt.ion() # Turn on interactive mode
        vis_fig, vis_ax = plt.subplots(1, 1, figsize=(6, 6)) # Create figure and axes
        vis_fig.suptitle(f"Episode {os.path.basename(episode_to_run)} Trajectory")

    # --- Inference Loop (per timestep) ---
    total_mse = 0.0
    timesteps_processed = 0

    try:
        with torch.no_grad():
            # Add frame skipping logic
            frame_skip = args.frame_skip
            processed_timesteps = episode_timesteps[::frame_skip] if frame_skip > 1 else episode_timesteps
            logging.info(f"Processing {len(processed_timesteps)} frames out of {len(episode_timesteps)} total frames (frame_skip={frame_skip})")

            for timestep_idx, step_data in enumerate(tqdm(processed_timesteps, desc=f"Episode {os.path.basename(episode_to_run)}", bar_format=TQDM_BAR_FORMAT)):
                # Spin ROS2 node once to process callbacks
                executor.spin_once(timeout_sec=0)

                # --- Get Ground Truth State ---
                try:
                    gt_angles = step_data['angles']
                    gt_gripper = step_data['gripper_value'][0]
                    gt_state_list = gt_angles + [float(gt_gripper)]
                    gt_state_tensor = torch.tensor(gt_state_list, dtype=torch.float32).unsqueeze(0).to(device) # Add batch dim
                except (KeyError, IndexError, TypeError, ValueError) as e:
                    logging.warning(f"Skipping timestep {timestep_idx} due to invalid state data: {e}. Data: {step_data}")
                    continue

                # --- Load Image (for visualization AND transformation) ---
                image_path = step_data.get("full_image_path")
                if not image_path or not os.path.isfile(image_path):
                     logging.warning(f"Skipping timestep {timestep_idx} due to missing or invalid image path: {image_path}")
                     continue
                try:
                    # Load PIL Image (used for visualization)
                    pil_image = Image.open(image_path).convert('RGB')
                    # Transform image for model input
                    image_tensor = image_transform(pil_image).unsqueeze(0).to(device) # Add batch dim
                except Exception as e:
                    logging.warning(f"Skipping timestep {timestep_idx} due to image load/transform error: {e}. Path: {image_path}")
                    continue

                # --- Visualize Current Frame ---
                if args.visualize_trajectory and vis_ax is not None:
                    vis_ax.clear() # Clear previous frame
                    vis_ax.imshow(pil_image) # Display the original PIL image
                    vis_ax.set_title(f"Timestep: {timestep_idx}")
                    vis_ax.axis('off') # Hide axes
                    plt.pause(0.01) # Brief pause to update display

                # --- Perform Sampling (batch size 1) ---
                predicted_state_batch = p_sample_loop(
                    model,
                    shape=(1, state_dim), # Batch size 1
                    timesteps=timesteps,
                    betas=betas,
                    sqrt_one_minus_alphas_cumprod=sqrt_one_minus_alphas_cumprod,
                    sqrt_recip_alphas=sqrt_recip_alphas,
                    posterior_variance=posterior_variance,
                    device=device,
                    image_input=image_tensor # Shape (1, C, H, W)
                )

                # --- Compare and Print ---
                gt_state_np = gt_state_tensor.squeeze(0).cpu().numpy()
                pred_state_np = predicted_state_batch.squeeze(0).cpu().numpy()
                mse_step = F.mse_loss(predicted_state_batch, gt_state_tensor).item()
                total_mse += mse_step
                timesteps_processed += 1

                print(f"\n--- Timestep {timestep_idx} (Episode {os.path.basename(episode_to_run)}) ---")
                np.set_printoptions(precision=4, suppress=True)
                print(f"  Ground Truth State: {gt_state_np}")
                print(f"  Predicted State:   {pred_state_np}")
                print(f"  MSE for timestep:  {mse_step:.6f}")

                # --- Send Commands to Gazebo ---
                if args.control_robot:
                    # Extract predicted joint angles and gripper value
                    predicted_angles = pred_state_np[:-1]  # All but last value
                    predicted_gripper = pred_state_np[-1]  # Last value

                    # Send commands to the robot
                    gazebo_node.send_arm_command(predicted_angles)
                    gazebo_node.send_gripper_command(predicted_gripper)

                    # Wait for the specified duration to allow robot to move
                    time.sleep(args.command_delay)

        # --- Final Results ---
        if timesteps_processed > 0:
            avg_mse = total_mse / timesteps_processed
            logging.info(f"Inference finished for episode {os.path.basename(episode_to_run)}. Processed {timesteps_processed} timesteps.")
            logging.info(f"Average State MSE over episode: {avg_mse:.6f}")
        else:
            logging.warning(f"Inference finished for episode {os.path.basename(episode_to_run)}, but no timesteps were processed successfully.")

    except KeyboardInterrupt:
        logging.info("Inference interrupted by user.")
    finally:
        # --- Cleanup ---
        if args.visualize_trajectory and vis_fig is not None:
            plt.ioff() # Turn off interactive mode
            vis_ax.clear()
            vis_ax.set_title(f"Episode {os.path.basename(episode_to_run)} Finished")
            vis_ax.text(0.5, 0.5, 'Trajectory Finished', horizontalalignment='center', verticalalignment='center', transform=vis_ax.transAxes)
            plt.show() # Keep the final window open until manually closed

        # Shutdown ROS2
        rclpy.shutdown()


# --- Main Execution Block ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Inference on an Episode and Visualize in Gazebo")

    # Required Argument
    parser.add_argument('--checkpoint_path', type=str, default='checkpoints/model_best.pth', help='Path to the trained model checkpoint (.pth file)')

    # Data Arguments
    parser.add_argument('--data_dir', type=str, default='../mycobot_episodes/', help='Base directory containing episode subdirectories')
    parser.add_argument('--episode_path', type=str, default=None, help='Specific episode directory path to run inference on (default: random)')

    # Inference Control Arguments
    parser.add_argument('--gpu_id', type=int, default=0, help='GPU ID to use if available')
    parser.add_argument('--visualize_trajectory', action='store_true', help='If set, display trajectory images during inference')
    parser.add_argument('--control_robot', action='store_true', help='If set, send commands to the robot in Gazebo')
    parser.add_argument('--command_delay', type=float, default=1.0, help='Delay between commands (seconds)')
    parser.add_argument('--frame_skip', type=int, default=2, help='Process every Nth frame (2 = half the frames, 3 = one-third, etc.)')

    # Optional: Arguments needed if checkpoint doesn't contain 'args' (fallback)
    parser.add_argument('--state_dim', type=int, default=7, help='Dimension of the state vector (if not in checkpoint)')
    parser.add_argument('--image_size', type=int, default=224, help='Image size used during training (if not in checkpoint)')
    parser.add_argument('--image_feature_dim', type=int, default=512, help='Image feature dimension (if not in checkpoint)')
    parser.add_argument('--time_emb_dim', type=int, default=64, help='Timestep embedding dimension (if not in checkpoint)')
    parser.add_argument('--hidden_dim', type=int, default=256, help='MLP hidden dimension (if not in checkpoint)')
    parser.add_argument('--num_mlp_layers', type=int, default=4, help='Number of MLP layers (if not in checkpoint)')
    parser.add_argument('--diffusion_timesteps', type=int, default=1000, help='Number of diffusion timesteps used during training (if not in checkpoint)')
    parser.add_argument('--beta_start', type=float, default=0.0001, help='Beta start value (if not in checkpoint)')
    parser.add_argument('--beta_end', type=float, default=0.02, help='Beta end value (if not in checkpoint)')

    args = parser.parse_args()

    run_inference_in_gazebo(args)
