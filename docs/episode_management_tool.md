# MyCobot Episodes Manager

This tool helps you manage your MyCobot episode recordings by checking for completeness and cleaning up bad episodes.

## Features

- **Scan Episodes**: Check if episodes have both a `frame_dir` folder and a `states.json` file
- **Clean Episodes**: Delete incomplete episodes, episodes marked as failed, and failure marker files
- **Automatic Detection**: Identifies episodes with missing components or failure indicators
- **Failure Marker Detection**: Identifies and cleans up `FAILED_EPISODE_*.txt` files

## Installation

The script is installed as a system-wide command `mycobot-episodes` for easy access.

## Usage

### Scanning Episodes

To check the status of all episodes:

```bash
mycobot-episodes scan
```

This will display:
- Total number of episodes found
- Number of complete episodes
- Number of incomplete episodes (missing frame_dir or states.json)
- Number of failed episodes (based on states.json content)
- Number of failure marker files (FAILED_EPISODE_*.txt)
- Lists of incomplete episodes, failed episodes, and failure marker files

### Cleaning Episodes

To delete incomplete episodes, failed episodes, and failure marker files:

```bash
mycobot-episodes clean
```

This will:
1. Scan all episodes and failure marker files
2. Identify incomplete episodes, failed episodes, and failure marker files
3. Ask for confirmation before deleting
4. Delete the episodes and failure marker files if confirmed

### Custom Episodes Directory

By default, the script looks for episodes in `/home/wangyukun/mycobot_episodes`.
To specify a different directory:

```bash
mycobot-episodes scan --dir /path/to/episodes
mycobot-episodes clean --dir /path/to/episodes
```

## How It Works

- **Complete Episode**: Has both a `frame_dir` folder and a `states.json` file
- **Incomplete Episode**: Missing either `frame_dir` or `states.json`
- **Failed Episode**: Has both components but is marked as failed in `states.json` or has fewer than 5 frames
- **Failure Marker**: A text file named `FAILED_EPISODE_*.txt` that indicates a failed episode

## Failure Detection Methods

The script uses multiple methods to identify failed episodes:
1. Checks for `FAILED_EPISODE_*.txt` files in the episodes directory
2. Checks if states.json is empty or has very few frames (less than 5)
3. Checks for failure indicators in the states.json content

## Source Code

The script is located at `/home/wangyukun/ros2_ws/check_episodes.py`
