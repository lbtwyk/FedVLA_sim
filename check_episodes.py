#!/usr/bin/env python3

import os
import sys
import json
import shutil
import argparse

DEFAULT_EPISODES_DIR = "/home/wangyukun/ros2_ws/mycobot_episodes_degrees"

def check_episodes(episodes_dir):
    """
    Check if each episode in the directory has both a frame_dir folder and a states.json file.

    Args:
        episodes_dir: Path to the directory containing episode folders

    Returns:
        Tuple of (complete_episodes, incomplete_episodes)
    """
    if not os.path.exists(episodes_dir):
        print(f"Error: Directory {episodes_dir} does not exist")
        sys.exit(1)

    episode_dirs = [d for d in os.listdir(episodes_dir)
                   if os.path.isdir(os.path.join(episodes_dir, d)) and d.startswith("episode_")]

    complete_episodes = []
    incomplete_episodes = []

    for episode in episode_dirs:
        episode_path = os.path.join(episodes_dir, episode)
        has_frame_dir = os.path.isdir(os.path.join(episode_path, "frame_dir"))
        has_states_json = os.path.isfile(os.path.join(episode_path, "states.json"))

        if has_frame_dir and has_states_json:
            complete_episodes.append(episode)
        else:
            missing = []
            if not has_frame_dir:
                missing.append("frame_dir")
            if not has_states_json:
                missing.append("states.json")
            incomplete_episodes.append((episode, missing))

    return complete_episodes, incomplete_episodes

def find_failed_episodes(episodes_dir, complete_episodes):
    """
    Check for episodes marked as failed using multiple detection methods.

    Args:
        episodes_dir: Path to the directory containing episode folders
        complete_episodes: List of episodes that have both frame_dir and states.json

    Returns:
        Tuple of (failed_episodes, failure_reasons) where failure_reasons is a dict
        mapping episode names to their failure reasons
    """
    failed_episodes = []
    failure_reasons = {}  # Maps episode name to reason for failure

    # 1. Check for FAILED_EPISODE_*.txt files in the episodes directory
    failed_txt_files = [f for f in os.listdir(episodes_dir)
                       if f.startswith("FAILED_EPISODE_") and f.endswith(".txt")]

    # Extract episode IDs and failure details from the failure marker files
    for txt_file in failed_txt_files:
        # Extract the timestamp part from the filename
        timestamp = txt_file.replace("FAILED_EPISODE_", "").replace(".txt", "")

        # Find the corresponding episode folder
        matching_episodes = [ep for ep in complete_episodes if timestamp in ep]

        # Read the failure marker file to extract detailed failure information
        failure_details = "Unknown failure (from marker file)"
        try:
            with open(os.path.join(episodes_dir, txt_file), 'r') as f:
                marker_content = f.read()

                # Try to extract the episode number
                episode_match = None
                for line in marker_content.split('\n'):
                    if line.startswith("Episode:"):
                        episode_match = line.strip()
                        break

                # Try to extract the failure category
                failure_categories = []
                in_error_section = False
                for line in marker_content.split('\n'):
                    if "CRITICAL ERROR MESSAGES:" in line:
                        in_error_section = True
                        continue
                    if in_error_section and line.endswith("errors:"):
                        category = line.replace(":", "").strip()
                        failure_categories.append(category)
                    if in_error_section and "RECOMMENDATION:" in line:
                        break

                if failure_categories:
                    failure_details = f"Critical errors in: {', '.join(failure_categories)}"
                elif episode_match:
                    failure_details = f"Failed episode {episode_match}"
        except Exception as e:
            print(f"Warning: Could not parse failure marker {txt_file}: {e}")

        if matching_episodes:
            for episode in matching_episodes:
                if episode not in failed_episodes:
                    failed_episodes.append(episode)
                    failure_reasons[episode] = failure_details
        else:
            print(f"Warning: Found failure marker {txt_file} but no matching episode folder")

    # 2. Check states.json files for additional failure indicators
    for episode in complete_episodes:
        # Skip episodes already marked as failed
        if episode in failed_episodes:
            continue

        episode_path = os.path.join(episodes_dir, episode)
        states_json_path = os.path.join(episode_path, "states.json")

        try:
            with open(states_json_path, 'r') as f:
                states_data = json.load(f)

                # Check if the file is empty or has very few frames (less than 5)
                if not states_data or len(states_data) < 5:
                    failed_episodes.append(episode)
                    failure_reasons[episode] = f"Insufficient data: only {len(states_data) if states_data else 0} frames"
                    continue

                # Check if any state has a failure indicator
                for state in states_data:
                    if isinstance(state, dict) and (
                        state.get('failed', False) or
                        state.get('status') == 'failed' or
                        'error' in state
                    ):
                        failed_episodes.append(episode)
                        error_msg = state.get('error', 'Unknown error in states.json')
                        failure_reasons[episode] = f"Error in states.json: {error_msg}"
                        break

        except (json.JSONDecodeError, IOError) as e:
            print(f"Warning: Could not parse states.json for {episode}: {e}")
            # Consider episodes with invalid JSON as failed
            failed_episodes.append(episode)
            failure_reasons[episode] = f"Invalid states.json: {str(e)}"

    return failed_episodes, failure_reasons

def delete_episodes(episodes_dir, episodes_to_delete):
    """
    Delete the specified episodes.

    Args:
        episodes_dir: Path to the directory containing episode folders
        episodes_to_delete: List of episode folder names to delete

    Returns:
        Number of successfully deleted episodes
    """
    deleted_count = 0

    for episode in episodes_to_delete:
        episode_path = os.path.join(episodes_dir, episode)
        try:
            shutil.rmtree(episode_path)
            print(f"Deleted: {episode}")
            deleted_count += 1
        except Exception as e:
            print(f"Error deleting {episode}: {e}")

    return deleted_count

def scan_episodes(episodes_dir):
    """
    Scan episodes and report their status with detailed failure reasons.

    Args:
        episodes_dir: Path to the directory containing episode folders
    """
    complete_episodes, incomplete_episodes = check_episodes(episodes_dir)
    failed_episodes, failure_reasons = find_failed_episodes(episodes_dir, complete_episodes)

    # Find FAILED_EPISODE_*.txt files
    failed_txt_files = [f for f in os.listdir(episodes_dir)
                       if f.startswith("FAILED_EPISODE_") and f.endswith(".txt")]

    print(f"\nChecking episodes in: {episodes_dir}")
    print(f"Total episodes found: {len(complete_episodes) + len(incomplete_episodes)}")
    print(f"Complete episodes: {len(complete_episodes)}")
    print(f"Incomplete episodes: {len(incomplete_episodes)}")
    print(f"Failed episodes: {len(failed_episodes)}")
    print(f"Failure marker files: {len(failed_txt_files)}")

    if incomplete_episodes:
        print("\nIncomplete episodes:")
        for episode, missing in incomplete_episodes:
            print(f"  - {episode}: Missing {', '.join(missing)}")

    if failed_episodes:
        print("\nFailed episodes (with failure reasons):")
        for episode in failed_episodes:
            reason = failure_reasons.get(episode, "Unknown failure reason")
            print(f"  - {episode}: {reason}")

    if failed_txt_files:
        print("\nFailure marker files:")
        for txt_file in failed_txt_files:
            # Try to extract episode number and timestamp for more informative output
            timestamp = txt_file.replace("FAILED_EPISODE_", "").replace(".txt", "")
            episode_info = ""
            try:
                with open(os.path.join(episodes_dir, txt_file), 'r') as f:
                    for line in f:
                        if line.startswith("Episode:"):
                            episode_info = f" - {line.strip()}"
                            break
            except:
                pass
            print(f"  - {txt_file}{episode_info}")

    return complete_episodes, incomplete_episodes, failed_episodes, failure_reasons

def clean_episodes(episodes_dir):
    """
    Delete incomplete and failed episodes after confirmation.
    Provides detailed information about what will be deleted.

    Args:
        episodes_dir: Path to the directory containing episode folders
    """
    # Get episode information with detailed failure reasons
    complete_episodes, incomplete_episodes, failed_episodes, failure_reasons = scan_episodes(episodes_dir)

    # Extract just the episode names from the incomplete_episodes tuples
    incomplete_episode_names = [episode for episode, _ in incomplete_episodes]
    episodes_to_delete = incomplete_episode_names + failed_episodes

    # Find FAILED_EPISODE_*.txt files
    failed_txt_files = [f for f in os.listdir(episodes_dir)
                       if f.startswith("FAILED_EPISODE_") and f.endswith(".txt")]

    if not episodes_to_delete and not failed_txt_files:
        print("\nNo episodes or failure markers to delete. All episodes are complete and successful.")
        return

    # Provide a summary of what will be deleted
    print("\n=== DELETION SUMMARY ===")
    print(f"Found {len(episodes_to_delete)} episodes to delete:")

    if incomplete_episode_names:
        print(f"  - {len(incomplete_episode_names)} incomplete episodes")

    if failed_episodes:
        print(f"  - {len(failed_episodes)} failed episodes")
        # Show a sample of failure reasons if there are many
        if len(failed_episodes) > 5:
            print("\nSample of failure reasons:")
            for episode in failed_episodes[:5]:
                reason = failure_reasons.get(episode, "Unknown failure reason")
                print(f"  - {episode}: {reason}")
            print(f"  ... and {len(failed_episodes) - 5} more")
        else:
            print("\nFailure reasons:")
            for episode in failed_episodes:
                reason = failure_reasons.get(episode, "Unknown failure reason")
                print(f"  - {episode}: {reason}")

    if failed_txt_files:
        print(f"\nFound {len(failed_txt_files)} failure marker files to delete.")

    # Ask for confirmation
    confirmation = input("\nDo you want to delete these episodes and failure markers? (yes/no): ")

    if confirmation.lower() in ['yes', 'y']:
        # Delete episodes
        deleted_count = delete_episodes(episodes_dir, episodes_to_delete)

        # Delete FAILED_EPISODE_*.txt files
        txt_deleted_count = 0
        for txt_file in failed_txt_files:
            try:
                os.remove(os.path.join(episodes_dir, txt_file))
                print(f"Deleted failure marker: {txt_file}")
                txt_deleted_count += 1
            except Exception as e:
                print(f"Error deleting failure marker {txt_file}: {e}")

        # Print summary
        print("\n=== DELETION RESULTS ===")
        print(f"Deleted {deleted_count} out of {len(episodes_to_delete)} episodes.")
        print(f"Deleted {txt_deleted_count} out of {len(failed_txt_files)} failure marker files.")

        # Calculate success rate
        successful_episodes = len(complete_episodes) - len(failed_episodes)
        total_episodes = len(complete_episodes) + len(incomplete_episodes)
        if total_episodes > 0:
            success_rate = (successful_episodes / total_episodes) * 100
            print(f"\nCurrent success rate: {success_rate:.1f}% ({successful_episodes} successful out of {total_episodes} total)")
    else:
        print("Operation cancelled. No episodes or failure markers were deleted.")

def main():
    parser = argparse.ArgumentParser(description='Manage mycobot episodes')
    parser.add_argument('action', choices=['scan', 'clean'],
                        help='Action to perform: scan (check episodes) or clean (delete bad episodes)')
    parser.add_argument('--dir', default=DEFAULT_EPISODES_DIR,
                        help=f'Episodes directory (default: {DEFAULT_EPISODES_DIR})')

    args = parser.parse_args()

    if args.action == 'scan':
        scan_episodes(args.dir)
    elif args.action == 'clean':
        clean_episodes(args.dir)

if __name__ == "__main__":
    main()
