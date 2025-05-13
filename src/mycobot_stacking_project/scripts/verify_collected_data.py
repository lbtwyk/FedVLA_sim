#!/usr/bin/env python3
"""
Script to verify the collected episode data.

This script checks that the collected data is complete and valid,
including verifying that all image files exist and that the JSON data
has the expected structure.
"""

import os
import sys
import json
import argparse
import cv2
import numpy as np
from datetime import datetime


def verify_episode_data(episode_path):
    """
    Verify the data for a single episode.
    
    Args:
        episode_path: Path to the episode directory
        
    Returns:
        True if the episode data is valid, False otherwise
    """
    print(f"\nVerifying episode: {os.path.basename(episode_path)}")
    
    # Check if states.json exists
    json_path = os.path.join(episode_path, "states.json")
    if not os.path.exists(json_path):
        print(f"  ERROR: states.json not found in {episode_path}")
        return False
    
    # Load the JSON data
    try:
        with open(json_path, 'r') as f:
            states = json.load(f)
    except Exception as e:
        print(f"  ERROR: Failed to load states.json: {e}")
        return False
    
    if not isinstance(states, list):
        print(f"  ERROR: states.json is not a list")
        return False
    
    if len(states) == 0:
        print(f"  ERROR: states.json is empty")
        return False
    
    print(f"  Found {len(states)} states in states.json")
    
    # Check the structure of the first state
    first_state = states[0]
    required_keys = ["angles", "gripper_value", "image"]
    for key in required_keys:
        if key not in first_state:
            print(f"  ERROR: Required key '{key}' not found in state")
            return False
    
    # Check that angles is a list of 6 floats
    if not isinstance(first_state["angles"], list) or len(first_state["angles"]) != 6:
        print(f"  ERROR: 'angles' should be a list of 6 floats, got {first_state['angles']}")
        return False
    
    # Check that gripper_value is a list with one integer
    if not isinstance(first_state["gripper_value"], list) or len(first_state["gripper_value"]) != 1:
        print(f"  ERROR: 'gripper_value' should be a list with one integer, got {first_state['gripper_value']}")
        return False
    
    # Check that image is a string
    if not isinstance(first_state["image"], str):
        print(f"  ERROR: 'image' should be a string, got {first_state['image']}")
        return False
    
    # Check that the image directory exists
    image_dir = os.path.join(episode_path, os.path.dirname(first_state["image"]))
    if not os.path.exists(image_dir):
        print(f"  ERROR: Image directory {image_dir} not found")
        return False
    
    # Check that all image files exist
    missing_images = 0
    for state in states:
        image_path = os.path.join(episode_path, state["image"])
        if not os.path.exists(image_path):
            missing_images += 1
            if missing_images <= 5:  # Only show the first 5 missing images
                print(f"  ERROR: Image file {image_path} not found")
    
    if missing_images > 0:
        print(f"  ERROR: {missing_images} image files are missing")
        return False
    
    # Display a sample image
    sample_index = len(states) // 2  # Middle of the episode
    sample_state = states[sample_index]
    sample_image_path = os.path.join(episode_path, sample_state["image"])
    
    try:
        image = cv2.imread(sample_image_path)
        if image is None:
            print(f"  ERROR: Failed to load sample image {sample_image_path}")
        else:
            print(f"  Sample image {sample_image_path} loaded successfully")
            print(f"  Image shape: {image.shape}")
            print(f"  Sample state angles: {sample_state['angles']}")
            print(f"  Sample state gripper_value: {sample_state['gripper_value']}")
    except Exception as e:
        print(f"  ERROR: Failed to load sample image: {e}")
    
    print(f"  Episode verification completed successfully")
    return True


def main():
    """Main function to verify collected data."""
    parser = argparse.ArgumentParser(description="Verify collected episode data")
    parser.add_argument("--data-dir", type=str, required=True,
                        help="Directory containing the collected episode data")
    
    args = parser.parse_args()
    
    # Check if the data directory exists
    if not os.path.exists(args.data_dir):
        print(f"ERROR: Data directory {args.data_dir} not found")
        sys.exit(1)
    
    # Find all episode directories
    episode_dirs = []
    for item in os.listdir(args.data_dir):
        item_path = os.path.join(args.data_dir, item)
        if os.path.isdir(item_path) and item.startswith("episode_"):
            episode_dirs.append(item_path)
    
    if not episode_dirs:
        print(f"ERROR: No episode directories found in {args.data_dir}")
        sys.exit(1)
    
    print(f"Found {len(episode_dirs)} episode directories")
    
    # Verify each episode
    valid_episodes = 0
    for episode_dir in sorted(episode_dirs):
        if verify_episode_data(episode_dir):
            valid_episodes += 1
    
    print(f"\nVerification complete: {valid_episodes}/{len(episode_dirs)} episodes are valid")
    
    if valid_episodes == len(episode_dirs):
        print("All episodes are valid!")
        return 0
    else:
        print(f"WARNING: {len(episode_dirs) - valid_episodes} episodes are invalid")
        return 1


if __name__ == "__main__":
    sys.exit(main())
