#!/usr/bin/env python3
# Modified version of inference.py to work with the existing data structure

import os
import torch
import torch.nn.functional as F
import torchvision.transforms as transforms
from tqdm import tqdm
import logging
import argparse
import math
import numpy as np
import random # For random episode selection
from PIL import Image # For loading images individually
from typing import Optional, List, Tuple, Union, Dict
import matplotlib.pyplot as plt # Import for visualization
import time # For pausing
import json # For loading JSON data
import glob # For finding episode directories

# Import custom modules (assuming they are in the same directory or accessible)
try:
    from dataset import RobotEpisodeDataset # Still needed to load episode data structure info
    from model import DiffusionPolicyModel
    # Import necessary helper functions (copied or imported from train.py)
    # Ensure train.py is accessible or copy these functions directly here
    from train import linear_beta_schedule, extract, p_sample, p_sample_loop
    # custom_collate_fn is no longer needed for this inference mode
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Please ensure dataset.py, model.py, and train.py (or the necessary functions) are accessible.")
    exit(1)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
# Suppress tqdm's default output slightly for cleaner inference logging
TQDM_BAR_FORMAT = '{l_bar}{bar}| {n_fmt}/{total_fmt} [{elapsed}<{remaining}]'


def load_episode_data(data_dir: str, episode_path: str) -> Optional[List[Dict]]:
    """Loads the states.json data for a specified episode directory."""
    # Use the full episode path instead of constructing it
    episode_dir = episode_path
    # Look for states.json (plural) instead of state.json
    state_file_path = os.path.join(episode_dir, 'states.json')

    if not os.path.isdir(episode_dir):
        logging.error(f"Episode directory not found: {episode_dir}")
        return None
    if not os.path.isfile(state_file_path):
        logging.error(f"states.json not found in {episode_dir}")
        return None

    try:
        with open(state_file_path, 'r') as f:
            episode_data = json.load(f)
        if not isinstance(episode_data, list):
            logging.error(f"states.json in {episode_dir} is not a list.")
            return None
        # Add full image path to each step for convenience
        for step_data in episode_data:
             if isinstance(step_data.get("image"), str):
                  step_data["full_image_path"] = os.path.join(episode_dir, step_data["image"])
             else:
                  logging.warning(f"Invalid image path found in {state_file_path} for step: {step_data}")
                  step_data["full_image_path"] = None # Mark as invalid

        return episode_data
    except json.JSONDecodeError:
        logging.error(f"Error decoding JSON from {state_file_path}")
        return None
    except Exception as e:
        logging.exception(f"An unexpected error occurred loading episode {episode_dir}: {e}")
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


def run_inference(args):
    """
    Loads a trained model and runs inference sequentially through a single episode,
    optionally visualizing the trajectory images.
    """
    # --- Device Setup ---
    device = torch.device(f"cuda:{args.gpu_id}" if torch.cuda.is_available() else "cpu")
    logging.info(f"Using device: {device}")

    # --- Load Checkpoint ---
    if not os.path.isfile(args.checkpoint_path):
        logging.error(f"Checkpoint file not found: {args.checkpoint_path}")
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
    sqrt_alphas_cumprod = torch.sqrt(alphas_cumprod)
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
            return
    else:
        # Select a random episode
        episode_to_run = random.choice(episode_dirs)

    logging.info(f"Selected episode: {episode_to_run}")

    # --- Load Data for Selected Episode ---
    episode_timesteps = load_episode_data(args.data_dir, episode_to_run)
    if episode_timesteps is None or not episode_timesteps:
        logging.error(f"Failed to load or empty data for episode {episode_to_run}.")
        return
    logging.info(f"Loaded {len(episode_timesteps)} timesteps for episode {os.path.basename(episode_to_run)}.")

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

    with torch.no_grad():
        for timestep_idx, step_data in enumerate(tqdm(episode_timesteps, desc=f"Episode {os.path.basename(episode_to_run)}", bar_format=TQDM_BAR_FORMAT)):

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
                plt.pause(args.vis_pause_duration) # Pause briefly to allow update and create animation effect

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
            print(f"  Predicted State:  {pred_state_np}")
            print(f"  MSE for timestep: {mse_step:.6f}")


    # --- Cleanup Visualization ---
    if args.visualize_trajectory and vis_fig is not None:
        plt.ioff() # Turn off interactive mode
        vis_ax.clear()
        vis_ax.set_title(f"Episode {os.path.basename(episode_to_run)} Finished")
        vis_ax.text(0.5, 0.5, 'Trajectory Finished', horizontalalignment='center', verticalalignment='center', transform=vis_ax.transAxes)
        plt.show() # Keep the final window open until manually closed

    # --- Final Results ---
    if timesteps_processed > 0:
        avg_mse = total_mse / timesteps_processed
        logging.info(f"Inference finished for episode {os.path.basename(episode_to_run)}. Processed {timesteps_processed} timesteps.")
        logging.info(f"Average State MSE over episode: {avg_mse:.6f}")
    else:
        logging.warning(f"Inference finished for episode {os.path.basename(episode_to_run)}, but no timesteps were processed successfully.")


# --- Main Execution Block ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Inference on a Single Episode Trajectory with Visualization")

    # Required Argument
    parser.add_argument('--checkpoint_path', type=str, default='checkpoints/model_best.pth', help='Path to the trained model checkpoint (.pth file)')

    # Data Arguments
    parser.add_argument('--data_dir', type=str, default='../mycobot_episodes/', help='Base directory containing episode subdirectories')
    parser.add_argument('--episode_path', type=str, default=None, help='Specific episode directory path to run inference on (default: random)')

    # Inference Control Arguments
    parser.add_argument('--gpu_id', type=int, default=0, help='GPU ID to use if available')
    parser.add_argument('--visualize_trajectory', action='store_true', help='If set, display trajectory images during inference') # Added visualization flag
    parser.add_argument('--vis_pause_duration', type=float, default=0.05, help='Pause duration (seconds) between frames for visualization') # Added pause duration

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

    run_inference(args)
