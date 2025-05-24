#!/usr/bin/env python3
"""
Model Inference Node for Cube Stacking

This node loads a trained diffusion policy model and runs inference on images
from a simulated camera, sending control commands to a simulated robot.
"""

import os
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, JointState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
import cv2
import torch
from PIL import Image as PILImage
import torchvision.transforms as transforms

# Add the path to the diffusion policy model
import importlib.util
import sys

class ModelInferenceNode(Node):
    """
    ROS 2 node for running inference with a trained diffusion policy model.

    This node:
    1. Loads a trained model from a checkpoint
    2. Subscribes to camera images from Gazebo
    3. Runs inference on those images
    4. Sends control commands to the robot
    """

    def __init__(self):
        """Initialize the model inference node."""
        super().__init__('model_inference_node')

        # Declare parameters
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('model_dir', '')
        self.declare_parameter('image_topic', '/camera_head/color/image_raw')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('inference_rate', 10.0)  # Hz
        self.declare_parameter('arm_joint_names', [
            'link1_to_link2', 'link2_to_link3', 'link3_to_link4',
            'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange'
        ])
        self.declare_parameter('gripper_joint_name', 'gripper_controller')

        # Get parameters
        self.checkpoint_path = self.get_parameter('checkpoint_path').value
        self.model_dir = self.get_parameter('model_dir').value
        self.image_topic = self.get_parameter('image_topic').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.inference_rate = self.get_parameter('inference_rate').value
        self.arm_joint_names = self.get_parameter('arm_joint_names').value
        self.gripper_joint_name = self.get_parameter('gripper_joint_name').value

        # Add model directory to path if provided
        if self.model_dir:
            # Expand the path if it contains a tilde
            expanded_model_dir = os.path.expanduser(self.model_dir)

            # Add the model directory to the beginning of sys.path to ensure it's found first
            sys.path.insert(0, expanded_model_dir)
            self.get_logger().info(f"Added {expanded_model_dir} to Python path")
            self.get_logger().info(f"Python path: {sys.path}")

            # List files in the model directory
            self.get_logger().info(f"Files in model directory: {os.listdir(expanded_model_dir)}")

            # Update the model_dir to use the expanded path
            self.model_dir = expanded_model_dir

        # Import the model and inference modules
        try:
            # Dynamic imports for the model and inference code
            self.get_logger().info("Importing model modules...")

            # Try to import the modules using importlib
            model_path = os.path.join(self.model_dir, "model.py")
            train_path = os.path.join(self.model_dir, "train.py")

            self.get_logger().info(f"Loading model from: {model_path}")
            self.get_logger().info(f"Loading train from: {train_path}")

            spec_model = importlib.util.spec_from_file_location("model", model_path)
            model_module = importlib.util.module_from_spec(spec_model)
            spec_model.loader.exec_module(model_module)

            spec_train = importlib.util.spec_from_file_location("train", train_path)
            train_module = importlib.util.module_from_spec(spec_train)
            spec_train.loader.exec_module(train_module)

            self.DiffusionPolicyModel = model_module.DiffusionPolicyModel
            self.linear_beta_schedule = train_module.linear_beta_schedule
            self.extract = train_module.extract
            self.p_sample = train_module.p_sample
            self.p_sample_loop = train_module.p_sample_loop

            self.get_logger().info("Successfully imported model modules")
        except Exception as e:
            self.get_logger().error(f"Error importing model modules: {e}")
            self.get_logger().error("Please ensure model.py and train.py are in the model_dir")
            import traceback
            self.get_logger().error(traceback.format_exc())
            rclpy.shutdown()
            sys.exit(1)

        # Initialize model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using device: {self.device}")
        self.model = None
        self.image_transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        # Load the model
        self.load_model()

        # Set up diffusion parameters
        self.setup_diffusion_params()

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Create callback groups
        self.timer_callback_group = ReentrantCallbackGroup()
        self.subscription_callback_group = MutuallyExclusiveCallbackGroup()
        self.action_callback_group = MutuallyExclusiveCallbackGroup()

        # Create subscribers
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
            callback_group=self.subscription_callback_group
        )

        self.joint_states_subscription = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_states_callback,
            10,
            callback_group=self.subscription_callback_group
        )

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

        # Create timer for inference
        self.inference_timer = self.create_timer(
            1.0 / self.inference_rate,
            self.inference_callback,
            callback_group=self.timer_callback_group
        )

        # Initialize state variables
        self.latest_image = None
        self.latest_joint_states = None
        self.is_running_inference = False

        self.get_logger().info("Model inference node initialized")

    def load_model(self):
        """Load the trained model from checkpoint."""
        # Expand the checkpoint path if it contains a tilde
        expanded_checkpoint_path = os.path.expanduser(self.checkpoint_path)

        if not expanded_checkpoint_path or not os.path.isfile(expanded_checkpoint_path):
            self.get_logger().error(f"Checkpoint file not found: {expanded_checkpoint_path}")
            rclpy.shutdown()
            sys.exit(1)

        self.get_logger().info(f"Loading checkpoint from: {expanded_checkpoint_path}")

        # Update the checkpoint_path to use the expanded path
        self.checkpoint_path = expanded_checkpoint_path
        try:
            # Explicitly set weights_only=False to handle numpy scalar objects in the checkpoint
            # This addresses the change in PyTorch 2.6 where weights_only defaults to True
            checkpoint = torch.load(self.checkpoint_path, map_location=self.device, weights_only=False)

            # Get model parameters from checkpoint
            train_args = checkpoint.get('args', {})
            if isinstance(train_args, dict):
                self.train_args = train_args
            else:
                self.train_args = vars(train_args)

            # Initialize model with ResNet34 backbone
            state_dim = self.train_args.get('state_dim', 7)  # Default: 6 joints + 1 gripper
            self.model = self.DiffusionPolicyModel(
                state_dim=state_dim,
                time_emb_dim=self.train_args.get('time_emb_dim', 64),
                hidden_dim=self.train_args.get('hidden_dim', 256),
                num_layers=self.train_args.get('num_mlp_layers', 4),
                image_feature_dim=self.train_args.get('image_feature_dim', 512),  # ResNet34 feature dimension
                use_pretrained_resnet=self.train_args.get('use_pretrained_resnet', True),
                freeze_resnet=self.train_args.get('freeze_resnet', True)
            ).to(self.device)

            # Load model weights
            self.model.load_state_dict(checkpoint['model_state_dict'])
            self.model.eval()
            self.get_logger().info("Successfully loaded model")
        except Exception as e:
            self.get_logger().error(f"Error loading model: {e}")
            rclpy.shutdown()
            sys.exit(1)

    def setup_diffusion_params(self):
        """Set up diffusion sampling parameters."""
        # Get diffusion parameters from train_args or use defaults
        self.timesteps = self.train_args.get('timesteps', 100)
        self.beta_start = self.train_args.get('beta_start', 1e-4)
        self.beta_end = self.train_args.get('beta_end', 0.02)

        # Calculate diffusion schedule
        self.betas = self.linear_beta_schedule(
            self.timesteps, self.beta_start, self.beta_end
        )

        # Pre-calculate diffusion parameters
        alphas = 1.0 - self.betas
        alphas_cumprod = torch.cumprod(alphas, dim=0)

        self.sqrt_recip_alphas = torch.sqrt(1.0 / alphas)
        self.sqrt_one_minus_alphas_cumprod = torch.sqrt(1.0 - alphas_cumprod)

        # Fix the tensor size mismatch issue
        if len(self.betas) == len(alphas_cumprod[:-1]):
            self.posterior_variance = self.betas * (1.0 - alphas_cumprod[:-1]) / (1.0 - alphas_cumprod[1:])
            # Add a zero at the end for the last timestep
            self.posterior_variance = torch.cat([self.posterior_variance, torch.tensor([0.0])])
        else:
            # Alternative calculation if sizes don't match
            self.get_logger().warning(f"Tensor size mismatch: betas={len(self.betas)}, alphas_cumprod[:-1]={len(alphas_cumprod[:-1])}")
            # Create posterior variance directly
            self.posterior_variance = torch.zeros_like(self.betas)
            # Calculate for all but the last timestep
            for t in range(len(self.betas) - 1):
                self.posterior_variance[t] = self.betas[t] * (1.0 - alphas_cumprod[t]) / (1.0 - alphas_cumprod[t+1])
            # Last timestep has zero variance
            self.posterior_variance[-1] = 0.0

        self.get_logger().info(f"Diffusion schedule set up with {self.timesteps} timesteps")

    def image_callback(self, msg):
        """Process incoming camera images."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def joint_states_callback(self, msg):
        """Process incoming joint states."""
        self.latest_joint_states = msg

    def inference_callback(self):
        """Run inference on the latest image and control the robot."""
        # Skip if already running inference or no image available
        if self.is_running_inference or self.latest_image is None:
            return

        self.is_running_inference = True

        try:
            # Convert OpenCV image to PIL Image
            pil_image = PILImage.fromarray(self.latest_image)

            # Transform image for model input
            image_tensor = self.image_transform(pil_image).unsqueeze(0).to(self.device)

            # Run inference
            with torch.no_grad():
                predicted_state_batch = self.p_sample_loop(
                    self.model,
                    shape=(1, self.train_args.get('state_dim', 7)),
                    timesteps=self.timesteps,
                    betas=self.betas,
                    sqrt_one_minus_alphas_cumprod=self.sqrt_one_minus_alphas_cumprod,
                    sqrt_recip_alphas=self.sqrt_recip_alphas,
                    posterior_variance=self.posterior_variance,
                    device=self.device,
                    image_input=image_tensor
                )

            # Extract predicted joint angles and gripper value
            predicted_state = predicted_state_batch[0].cpu().numpy()
            predicted_angles = predicted_state[:-1]  # All but last value
            predicted_gripper = predicted_state[-1]  # Last value

            # Send commands to the robot
            self.send_arm_command(predicted_angles)
            self.send_gripper_command(predicted_gripper)

            self.get_logger().info(f"Sent commands - Angles: {predicted_angles}, Gripper: {predicted_gripper}")
        except Exception as e:
            self.get_logger().error(f"Error during inference: {e}")
        finally:
            self.is_running_inference = False

    def send_arm_command(self, angles):
        """Send a command to move the robot arm to specified joint positions."""
        # Convert angles from degrees to radians for robot control
        angles_rad = [angle * 3.14159265359 / 180.0 for angle in angles]

        # Create a trajectory point with the target positions
        point = JointTrajectoryPoint()
        point.positions = angles_rad
        point.time_from_start = Duration(sec=1, nanosec=0)  # 1 second for movement

        # Create and send the goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joint_names
        goal_msg.trajectory.points = [point]

        # Send the goal
        self.arm_client.wait_for_server()
        self.arm_client.send_goal_async(goal_msg)

    def send_gripper_command(self, gripper_value):
        """Send a command to the gripper."""
        # Map gripper value (0-100) to position (-0.7 to 0.0)
        # 0 = open, 100 = closed
        position = -0.7 * (gripper_value / 100.0)

        # Create and send the goal message
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 5.0

        # Send the goal
        self.gripper_client.wait_for_server()
        self.gripper_client.send_goal_async(goal_msg)


def main(args=None):
    """Run the model inference node."""
    rclpy.init(args=args)

    # Create the node
    node = ModelInferenceNode()

    # Use a MultiThreadedExecutor to enable parallel processing
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # Spin the node
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
