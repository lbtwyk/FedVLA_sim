#!/usr/bin/env python3
# Real-time inference using camera images from Gazebo

import os
import torch
import torch.nn.functional as F
import torchvision.transforms as transforms
import logging
import argparse
import numpy as np
import time
import sys
from PIL import Image
import matplotlib.pyplot as plt

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# ROS2 message types
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge

# Import custom modules will be done after parsing arguments to use model_dir

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class GazeboInferenceNode(Node):
    """ROS2 node for running inference and controlling the robot in Gazebo."""

    def __init__(self, args):
        """Initialize the node with the provided arguments."""
        super().__init__('gazebo_inference_node')

        # Store arguments
        self.args = args

        # Initialize CV bridge for image conversion
        self.cv_bridge = CvBridge()

        # Initialize image storage
        self.latest_image = None
        self.latest_image_tensor = None
        self.image_lock = False  # Simple lock to prevent concurrent image processing

        # Create callback groups
        self.action_callback_group = ReentrantCallbackGroup()
        self.subscription_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = ReentrantCallbackGroup()

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

        # Create camera image subscriber
        self.image_subscription = self.create_subscription(
            ROSImage,
            '/camera_head/color/image_raw',  # Topic for the RealSense camera
            self.image_callback,
            10,  # QoS depth
            callback_group=self.subscription_callback_group
        )

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

        # Initialize arm to starting position if specified
        self.initialized = False
        if hasattr(args, 'initial_arm_position') and args.initial_arm_position:
            self.get_logger().info(f"Moving arm to initial position: {args.initial_arm_position}")
            self.move_to_initial_position()

        # Create timer for inference if rate is specified
        if args.inference_rate > 0:
            self.inference_timer = self.create_timer(
                1.0 / args.inference_rate,  # Convert Hz to seconds
                self.inference_callback,
                callback_group=self.timer_callback_group
            )
            self.get_logger().info(f"Created inference timer with rate {args.inference_rate} Hz")
        else:
            self.inference_timer = None
            self.get_logger().info("No inference timer created (rate set to 0)")

    def move_to_initial_position(self):
        """Move the arm to the specified initial position."""
        try:
            # Send arm to initial position
            self.send_arm_command(self.args.initial_arm_position)
            self.get_logger().info("Arm moving to initial position...")

            # Give time for the movement to complete
            time.sleep(3.0)
            self.initialized = True
            self.get_logger().info("Arm initialization complete!")

        except Exception as e:
            self.get_logger().error(f"Failed to move arm to initial position: {e}")

    def calculate_forward_position(self, target_height=0.3):
        """
        Calculate joint angles for a forward-bent position at the specified height.

        This is a simplified inverse kinematics calculation for the MyCobot 280.
        For a more accurate calculation, you would use the robot's kinematics.

        Args:
            target_height: Desired Z-height of the end effector in meters

        Returns:
            List of 6 joint angles in radians
        """
        # These are approximate values for a forward-bent position
        # at roughly 0.3m height. Adjust based on testing.

        # Starting from the ready pose and modifying for desired height
        joint_angles = [
            0.0,        # link1_to_link2: Base rotation (no rotation)
            0.5,        # link2_to_link3: Shoulder pitch (forward tilt)
            1.2,        # link3_to_link4: Elbow bend (forward)
            1.0,        # link4_to_link5: Wrist pitch (adjust for height)
            0.0,        # link5_to_link6: Wrist roll (no rotation)
            0.0         # link6_to_link6_flange: Tool rotation (no rotation)
        ]

        self.get_logger().info(f"Calculated forward position for height {target_height}m: {joint_angles}")
        return joint_angles

    def image_callback(self, msg):
        """Process incoming camera images."""
        if self.image_lock:
            return  # Skip if we're already processing an image

        try:
            # Store the timestamp for freshness checking
            self.latest_image_stamp = msg.header.stamp

            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Convert to PIL Image for visualization and transformation
            self.latest_image = Image.fromarray(cv_image)

            # Log image dimensions on first receipt
            if hasattr(self, 'image_dims_logged') is False:
                self.get_logger().info(f"Received camera image with dimensions: {self.latest_image.size}")
                self.image_dims_logged = True

            # Log that we received a new image (with timestamp)
            if hasattr(self, 'last_logged_stamp') is False or self.last_logged_stamp != self.latest_image_stamp:
                self.get_logger().debug(f"New camera image received with timestamp: {self.latest_image_stamp.sec}.{self.latest_image_stamp.nanosec}")
                self.last_logged_stamp = self.latest_image_stamp

        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")

    def inference_callback(self):
        """Run inference on the latest camera image."""
        # Skip if no image available or already processing
        if self.latest_image is None or self.image_lock:
            return

        # Check if we've already processed this image
        if hasattr(self, 'last_processed_stamp') and hasattr(self, 'latest_image_stamp'):
            if self.last_processed_stamp == self.latest_image_stamp:
                # Skip processing the same image again
                return

        self.image_lock = True

        try:
            # Store the timestamp of the image we're processing
            if hasattr(self, 'latest_image_stamp'):
                self.last_processed_stamp = self.latest_image_stamp
                self.get_logger().debug(f"Processing image with timestamp: {self.last_processed_stamp.sec}.{self.last_processed_stamp.nanosec}")

            # Transform the latest image for the model
            # First transform to tensor on CPU, then move to the correct device
            image_tensor = self.args.image_transform(self.latest_image).unsqueeze(0).to(self.args.device)

            # Store the current image for visualization in the main thread
            if self.args.visualize_trajectory:
                try:
                    # Use numpy array for safer copying between threads
                    self.args.current_image = np.array(self.latest_image)
                    # Signal that a new image is available
                    self.args.new_image_available = True
                except Exception as e:
                    self.get_logger().warning(f"Error copying image for visualization: {e}")

            # Run inference with the model
            inference_start = time.time()
            with torch.no_grad():
                # Ensure all tensors are on the same device
                device = self.args.device
                model = self.args.model

                # Create a shape tuple for the desired output
                shape = (1, self.args.state_dim)

                # Get diffusion parameters and ensure they're on the correct device
                timesteps = self.args.timesteps
                betas = self.args.betas.to(device)
                sqrt_one_minus_alphas_cumprod = self.args.sqrt_one_minus_alphas_cumprod.to(device)
                sqrt_recip_alphas = self.args.sqrt_recip_alphas.to(device)
                posterior_variance = self.args.posterior_variance.to(device)

                # Ensure image_tensor is on the correct device
                image_input = image_tensor.to(device)

                # Call p_sample_loop with all tensors on the same device
                predicted_state_batch = p_sample_loop(
                    model,
                    shape=shape,
                    timesteps=timesteps,
                    betas=betas,
                    sqrt_one_minus_alphas_cumprod=sqrt_one_minus_alphas_cumprod,
                    sqrt_recip_alphas=sqrt_recip_alphas,
                    posterior_variance=posterior_variance,
                    device=device,
                    image_input=image_input
                )
            inference_time = time.time() - inference_start

            # Extract predicted joint angles and gripper value
            pred_state_np = predicted_state_batch.squeeze(0).cpu().numpy()
            predicted_angles = pred_state_np[:-1]  # All but last value
            predicted_gripper = pred_state_np[-1]  # Last value

            # Store prediction for visualization
            if self.args.visualize_trajectory:
                self.args.current_prediction = pred_state_np
                self.args.inference_time = inference_time

            # Print prediction
            np.set_printoptions(precision=4, suppress=True)
            self.get_logger().info(f"Predicted State: {pred_state_np} (inference took {inference_time:.3f}s)")

            # Send commands to the robot if enabled
            if self.args.control_robot:
                self.send_arm_command(predicted_angles)
                self.send_gripper_command(predicted_gripper)

                # Wait for the specified duration to allow robot to move
                if self.args.command_delay > 0:
                    time.sleep(self.args.command_delay)

        except Exception as e:
            self.get_logger().error(f"Error during inference: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.image_lock = False

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


def run_inference_in_gazebo(args):
    """
    Loads a trained model and runs real-time inference on camera images,
    sending the predicted states to the robot in Gazebo.
    """
    # --- Initialize ROS2 ---
    rclpy.init(args=None)

    # --- Device Setup ---
    device = torch.device(f"cuda:{args.gpu_id}" if torch.cuda.is_available() else "cpu")
    logging.info(f"Using device: {device}")
    args.device = device  # Store in args for easy access

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
        train_args = vars(args)  # Use current args as fallback
    elif isinstance(train_args, argparse.Namespace):
        train_args = vars(train_args)
    logging.info("Loaded training arguments from checkpoint.")

    # --- Diffusion Schedule Setup ---
    timesteps = train_args.get('diffusion_timesteps', args.diffusion_timesteps)
    beta_start = train_args.get('beta_start', args.beta_start)
    beta_end = train_args.get('beta_end', args.beta_end)

    # Create schedule on CPU first
    betas = linear_beta_schedule(timesteps=timesteps, beta_start=beta_start, beta_end=beta_end)
    alphas = 1. - betas
    alphas_cumprod = torch.cumprod(alphas, axis=0)
    alphas_cumprod_prev = F.pad(alphas_cumprod[:-1], (1, 0), value=1.0)
    sqrt_one_minus_alphas_cumprod = torch.sqrt(1. - alphas_cumprod)
    sqrt_recip_alphas = torch.sqrt(1.0 / alphas)
    posterior_variance = betas * (1. - alphas_cumprod_prev) / (1. - alphas_cumprod)
    logging.info(f"Diffusion schedule set up with {timesteps} timesteps.")

    # Store diffusion parameters in args for easy access - keep on CPU until needed
    # This prevents device mismatch errors when tensors are moved between devices
    args.timesteps = timesteps
    args.betas = betas.to('cpu')  # Keep on CPU until needed
    args.sqrt_one_minus_alphas_cumprod = sqrt_one_minus_alphas_cumprod.to('cpu')
    args.sqrt_recip_alphas = sqrt_recip_alphas.to('cpu')
    args.posterior_variance = posterior_variance.to('cpu')

    # --- Model Initialization ---
    if train_args:
        logging.info("Loaded training arguments from checkpoint.")
        args.state_dim = train_args.get('state_dim', args.state_dim)

        # Get image_feature_dim from checkpoint
        # image_feature_dim from checkpoint is the authority on what the model was trained with.
        loaded_image_feature_dim = train_args.get('image_feature_dim', None)

        if loaded_image_feature_dim is not None:
            logging.info(f"Using image_feature_dim {loaded_image_feature_dim} from checkpoint.")
        else:
            # If we can't determine from the checkpoint, use the default for ResNet34
            loaded_image_feature_dim = 512
            logging.info(f"No image_feature_dim found in checkpoint. Using default value for ResNet34: {loaded_image_feature_dim}")

        # Store the image_feature_dim from the checkpoint
        args.image_feature_dim_from_checkpoint = loaded_image_feature_dim

        args.time_emb_dim = train_args.get('time_emb_dim', args.time_emb_dim if hasattr(args, 'time_emb_dim') else 64)
        args.time_emb_mult = train_args.get('time_emb_mult', args.time_emb_mult if hasattr(args, 'time_emb_mult') else 4)
        args.hidden_dim = train_args.get('hidden_dim', args.hidden_dim if hasattr(args, 'hidden_dim') else 128)
        args.num_layers = train_args.get('num_layers', args.num_layers if hasattr(args, 'num_layers') else 4)

    else:
        logging.warning("No training arguments found in checkpoint. Using command-line/default args for model.")

        # Set image_feature_dim to 512 (ResNet34) if not explicitly provided
        if not hasattr(args, 'image_feature_dim') or args.image_feature_dim is None:
            args.image_feature_dim = 512  # Default for ResNet34
            logging.info(f"Setting image_feature_dim to {args.image_feature_dim} for ResNet34")

        args.image_feature_dim_from_checkpoint = args.image_feature_dim


    # Initialize model
    logging.info(f"Initializing model with ResNet34 backbone, image_feature_dim from checkpoint: {args.image_feature_dim_from_checkpoint}")
    logging.info(f"Time embedding dim: {args.time_emb_dim}, Time embedding mult: {args.time_emb_mult}, Hidden dim: {args.hidden_dim}")

    # Check if model.py supports time_emb_mult parameter
    import inspect
    model_init_params = inspect.signature(DiffusionPolicyModel.__init__).parameters

    # Determine the correct time_emb_mult from the checkpoint
    # This is critical for proper model loading
    checkpoint_time_emb_mult = None
    if 'model_state_dict' in checkpoint:
        if 'time_mlp.1.weight' in checkpoint['model_state_dict']:
            time_emb_dim = args.time_emb_dim
            weight_shape = checkpoint['model_state_dict']['time_mlp.1.weight'].shape
            if weight_shape[0] == time_emb_dim * 4:
                checkpoint_time_emb_mult = 4
            elif weight_shape[0] == time_emb_dim * 2:
                checkpoint_time_emb_mult = 2
            else:
                # Try to infer from dimensions
                checkpoint_time_emb_mult = weight_shape[0] // time_emb_dim
                if checkpoint_time_emb_mult <= 0:
                    checkpoint_time_emb_mult = 2  # Default to 2 if we can't determine

            logging.info(f"Detected time_emb_mult={checkpoint_time_emb_mult} from checkpoint")
            # Override the args value with the one from the checkpoint
            args.time_emb_mult = checkpoint_time_emb_mult

    model_kwargs = {
        'state_dim': args.state_dim,
        'image_feature_dim': args.image_feature_dim_from_checkpoint,  # Pass the one from checkpoint (can be None)
        'time_emb_dim': args.time_emb_dim,
        'hidden_dim': args.hidden_dim,
        'num_layers': args.num_layers,
        'use_pretrained_resnet': train_args.get('use_pretrained_resnet', True) if train_args else True,
        'freeze_resnet': train_args.get('freeze_resnet', True) if train_args else True
    }

    # Add time_emb_mult if the model supports it
    if 'time_emb_mult' in model_init_params:
        model_kwargs['time_emb_mult'] = args.time_emb_mult
        logging.info(f"Model supports time_emb_mult parameter, using value: {args.time_emb_mult}")

    model = DiffusionPolicyModel(**model_kwargs).to(device)

    logging.info(f"Loading model state dict...")
    try:
        # Check if the model has a custom load_state_dict method for handling legacy checkpoints
        if hasattr(model, 'load_state_dict_with_conversion') and callable(getattr(model, 'load_state_dict_with_conversion')):
            logging.info("Using custom state dict conversion method to handle architecture mismatch...")
            model.load_state_dict_with_conversion(checkpoint['model_state_dict'], strict=False)
        else:
            # Try direct loading first
            try:
                model.load_state_dict(checkpoint['model_state_dict'])
            except Exception as direct_load_error:
                logging.warning(f"Direct loading failed: {direct_load_error}")
                logging.info("Attempting to load with strict=False...")
                # Try loading with strict=False as a fallback
                model.load_state_dict(checkpoint['model_state_dict'], strict=False)
                logging.warning("Model loaded with strict=False. Some parameters may be missing or unexpected.")

        logging.info("Successfully loaded model state dict.")
    except Exception as e:
        logging.exception(f"Error loading state dict: {e}")
        rclpy.shutdown()
        return
    model.eval()
    args.model = model  # Store in args for easy access

    # --- Image Transform ---
    image_size = train_args.get('image_size', args.image_size)
    image_transform = transforms.Compose([
        transforms.Resize((image_size, image_size)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])
    args.image_transform = image_transform  # Store in args for easy access

    # --- Visualization Setup ---
    vis_fig, vis_ax = None, None
    if args.visualize_trajectory:
        logging.info("Visualization enabled. Make sure matplotlib window is visible.")
        plt.ion()  # Turn on interactive mode
        vis_fig, vis_ax = plt.subplots(1, 1, figsize=(6, 6))  # Create figure and axes
        vis_fig.suptitle("Real-time Diffusion Policy Inference")
        args.vis_fig = vis_fig  # Store in args for easy access
        args.vis_ax = vis_ax  # Store in args for easy access

    # --- Create ROS2 Node ---
    gazebo_node = GazeboInferenceNode(args)
    executor = MultiThreadedExecutor()
    executor.add_node(gazebo_node)

    # Initialize visualization variables
    if args.visualize_trajectory:
        args.current_image = None
        args.current_prediction = None
        args.new_image_available = False

    # --- Main Loop ---
    try:
        logging.info("Starting real-time inference. Press Ctrl+C to stop.")

        # Create a separate thread for ROS2 execution
        import threading
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()

        # Main thread handles visualization
        while rclpy.ok():
            if args.visualize_trajectory and args.new_image_available and args.current_image is not None:
                # Update the visualization in the main thread
                args.vis_ax.clear()
                args.vis_ax.imshow(args.current_image)

                # Add timestamp to title if available
                title = "Real-time Camera Feed"
                if hasattr(args, 'inference_time'):
                    title += f" (Inference: {args.inference_time:.3f}s)"
                args.vis_ax.set_title(title)

                # Display prediction if available
                if hasattr(args, 'current_prediction') and args.current_prediction is not None:
                    # Format the prediction array nicely
                    angles = args.current_prediction[:-1]
                    gripper = args.current_prediction[-1]

                    # Create a more readable prediction string
                    pred_str = f"Angles: {np.array2string(angles, precision=3, suppress_small=True)}\n"
                    pred_str += f"Gripper: {gripper:.2f}"

                    # Display the prediction
                    args.vis_ax.text(0.5, 0.95, pred_str,
                                    horizontalalignment='center',
                                    verticalalignment='top',
                                    transform=args.vis_ax.transAxes,
                                    bbox=dict(facecolor='white', alpha=0.7))

                args.vis_ax.axis('off')
                plt.pause(0.01)  # Brief pause to update display
                args.new_image_available = False

            # Sleep to avoid consuming too much CPU
            time.sleep(0.01)

    except KeyboardInterrupt:
        logging.info("Inference interrupted by user.")
    finally:
        # --- Cleanup ---
        if args.visualize_trajectory and vis_fig is not None:
            plt.ioff()  # Turn off interactive mode
            vis_ax.clear()
            vis_ax.set_title("Real-time Inference Finished")
            vis_ax.text(0.5, 0.5, 'Inference Finished', horizontalalignment='center',
                       verticalalignment='center', transform=vis_ax.transAxes)
            plt.show()  # Keep the final window open until manually closed

        # Shutdown ROS2
        rclpy.shutdown()


# --- Main Execution Block ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Real-time Inference with Camera Images and Visualize in Gazebo")

    # Required Arguments
    parser.add_argument('--checkpoint_path', type=str, default='checkpoints/model_best.pth',
                       help='Path to the trained model checkpoint (.pth file)')
    parser.add_argument('--model_dir', type=str, default='.',
                       help='Path to the directory containing model.py and train.py')

    # Inference Control Arguments
    parser.add_argument('--gpu_id', type=int, default=0, help='GPU ID to use if available')
    parser.add_argument('--visualize_trajectory', action='store_true',
                       help='If set, display camera images during inference')
    parser.add_argument('--control_robot', action='store_true',
                       help='If set, send commands to the robot in Gazebo')
    parser.add_argument('--command_delay', type=float, default=0.1,
                       help='Delay between commands (seconds)')
    parser.add_argument('--inference_rate', type=float, default=5.0,
                       help='Target inference rate in Hz (0 for max speed)')

    # Initial Position Arguments
    parser.add_argument('--initial_position', type=str, choices=['home', 'ready', 'forward', 'custom'],
                       default='home', help='Initial arm position: home (all zeros), ready (preset), forward (calculated), or custom')
    parser.add_argument('--initial_height', type=float, default=0.3,
                       help='Target height for forward position in meters (default: 0.3)')
    parser.add_argument('--custom_joints', type=float, nargs=6, metavar=('J1', 'J2', 'J3', 'J4', 'J5', 'J6'),
                       help='Custom joint angles in radians for each of the 6 joints')

    # Optional: Arguments needed if checkpoint doesn't contain 'args' (fallback)
    parser.add_argument('--state_dim', type=int, default=7,
                       help='Dimension of the state vector (if not in checkpoint)')
    parser.add_argument('--image_size', type=int, default=224,
                       help='Image size used during training (if not in checkpoint)')
    parser.add_argument('--image_feature_dim', type=int, default=512,
                       help='Image feature dimension (if not in checkpoint)')
    parser.add_argument('--time_emb_dim', type=int, default=64,
                       help='Timestep embedding dimension (if not in checkpoint)')
    parser.add_argument('--time_emb_mult', type=int, default=4,
                       help='Multiplier for time embedding MLP hidden dimension (default: 4)')
    parser.add_argument('--hidden_dim', type=int, default=128,
                       help='MLP hidden dimension (if not in checkpoint)')
    parser.add_argument('--num_mlp_layers', type=int, default=4,
                       help='Number of MLP layers (if not in checkpoint)')
    parser.add_argument('--diffusion_timesteps', type=int, default=1000,
                       help='Number of diffusion timesteps used during training (if not in checkpoint)')
    parser.add_argument('--beta_start', type=float, default=0.0001,
                       help='Beta start value (if not in checkpoint)')
    parser.add_argument('--beta_end', type=float, default=0.02,
                       help='Beta end value (if not in checkpoint)')

    args = parser.parse_args()

    # Process initial position arguments
    if args.initial_position == 'home':
        args.initial_arm_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    elif args.initial_position == 'ready':
        # From SRDF file: ready position with 90-degree bends
        args.initial_arm_position = [0.0, 0.0, 1.5708, 1.5708, 0.0, 0.0]
    elif args.initial_position == 'forward':
        # Calculate forward position based on target height
        # These are approximate values for forward-bent position
        if args.initial_height == 0.3:
            # Optimized for 0.3m height - bent forward (positive direction)
            args.initial_arm_position = [0.0, 0.5, 1.2, 1.0, 0.0, 0.0]
        else:
            # Basic scaling for other heights (this is approximate)
            base_height = 0.3
            height_factor = args.initial_height / base_height
            args.initial_arm_position = [
                0.0,                    # Base rotation
                0.5 * height_factor,    # Shoulder pitch (positive for forward)
                1.2,                    # Elbow bend
                1.0 * height_factor,    # Wrist pitch
                0.0,                    # Wrist roll
                0.0                     # Tool rotation
            ]
    elif args.initial_position == 'custom':
        if args.custom_joints is None:
            print("Error: --custom_joints must be specified when using --initial_position custom")
            exit(1)
        args.initial_arm_position = list(args.custom_joints)
    else:
        args.initial_arm_position = None

    print(f"Initial arm position set to: {args.initial_arm_position}")

    # Add model directory to Python path
    if args.model_dir:
        import sys
        model_dir = os.path.expanduser(args.model_dir)
        if model_dir not in sys.path:
            sys.path.insert(0, model_dir)
            print(f"Added {model_dir} to Python path")

    # Import custom modules after setting up the path
    try:
        from model import DiffusionPolicyModel
        from train import linear_beta_schedule, p_sample_loop
        print("Successfully imported model modules")
    except ImportError as e:
        print(f"Error importing modules: {e}")
        print(f"Please ensure model.py and train.py are in the directory: {args.model_dir}")
        exit(1)

    run_inference_in_gazebo(args)
