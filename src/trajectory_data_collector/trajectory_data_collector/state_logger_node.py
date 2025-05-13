#!/usr/bin/env python3
"""
State Logger Node for collecting trajectory data.

This node subscribes to sensor topics, captures data at a defined frequency
when recording is active, and saves it into the specified directory structure.
"""

import os
import json
from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from trajectory_data_interfaces.srv import StartStopEpisode
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class StateLoggerNode(Node):
    """
    ROS 2 node for logging robot state data for diffusion policy training.

    This node subscribes to joint states and camera images, and saves them
    in a structured format when recording is active.
    """

    def __init__(self):
        """Initialize the state logger node."""
        super().__init__('state_logger_node')

        # Declare parameters
        self.declare_parameter('output_base_dir', '/tmp/mycobot_episodes')
        self.declare_parameter('log_frequency_hz', 10.0)
        self.declare_parameter('arm_joint_names',
                              ['link1_to_link2', 'link2_to_link3', 'link3_to_link4', 'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange'])
        self.declare_parameter('gripper_joint_name', 'gripper_controller')
        self.declare_parameter('image_dir_name', 'frame_dir')
        self.declare_parameter('gripper_value_min_rad', -0.5)  # Closed position
        self.declare_parameter('gripper_value_max_rad', 0.0)   # Open position
        self.declare_parameter('gripper_value_output_min_int', 0)
        self.declare_parameter('gripper_value_output_max_int', 100)

        # Get parameters
        output_base_dir = self.get_parameter('output_base_dir').value
        # Expand the tilde in the path if it exists
        if output_base_dir.startswith('~'):
            self.output_base_dir = os.path.expanduser(output_base_dir)
        else:
            self.output_base_dir = output_base_dir

        self.log_frequency_hz = self.get_parameter('log_frequency_hz').value
        self.arm_joint_names = self.get_parameter('arm_joint_names').value
        self.gripper_joint_name = self.get_parameter('gripper_joint_name').value
        self.image_dir_name = self.get_parameter('image_dir_name').value
        self.gripper_value_min_rad = self.get_parameter('gripper_value_min_rad').value
        self.gripper_value_max_rad = self.get_parameter('gripper_value_max_rad').value
        self.gripper_value_output_min_int = self.get_parameter('gripper_value_output_min_int').value
        self.gripper_value_output_max_int = self.get_parameter('gripper_value_output_max_int').value

        # Initialize CV Bridge
        self.cv_bridge = CvBridge()

        # State variables
        self.is_recording = False
        self.current_episode_id = None
        self.current_episode_path = None
        self.current_image_dir_path = None
        self.current_episode_data_list = []
        self.frame_counter = 0

        # Latest messages
        self.latest_joint_states_msg = None
        self.latest_image_msg = None

        # Create callback groups for subscribers and timer
        self.subscriber_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        # Create subscribers
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10,
            callback_group=self.subscriber_callback_group
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera_head/color/image_raw',
            self.image_callback,
            10,
            callback_group=self.subscriber_callback_group
        )

        # Create timer for periodic logging
        log_period_sec = 1.0 / self.log_frequency_hz
        self.timer = self.create_timer(
            log_period_sec,
            self.log_timer_callback,
            callback_group=self.timer_callback_group
        )

        # Create service server
        self.service = self.create_service(
            StartStopEpisode,
            '~/start_stop_episode',
            self.handle_start_stop_service,
            callback_group=self.service_callback_group
        )

        self.get_logger().info('State Logger Node initialized')
        self.get_logger().info(f'Output directory: {self.output_base_dir}')
        self.get_logger().info(f'Log frequency: {self.log_frequency_hz} Hz')
        self.get_logger().info(f'Arm joint names: {self.arm_joint_names}')
        self.get_logger().info(f'Gripper joint name: {self.gripper_joint_name}')

    def joint_states_callback(self, msg):
        """Store the latest joint states message."""
        self.latest_joint_states_msg = msg

        # Print joint names when we receive a message
        if not hasattr(self, 'joint_names_printed') or not self.joint_names_printed:
            self.get_logger().info(f"Received joint states with names: {msg.name}")
            self.joint_names_printed = True

    def image_callback(self, msg):
        """Store the latest image message."""
        self.latest_image_msg = msg

    def log_timer_callback(self):
        """
        Periodically log the robot state when recording is active.

        This callback is triggered at the specified frequency and saves
        the current joint states and camera image to disk.
        """
        if not self.is_recording:
            return

        if self.latest_joint_states_msg is None:
            self.get_logger().warn('No joint states received yet')
            return

        if self.latest_image_msg is None:
            self.get_logger().warn('No images received yet')
            return

        # Process Image
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image_msg, 'bgr8')
            image_filename = f"image_{self.frame_counter:05d}.png"
            image_save_path = os.path.join(self.current_image_dir_path, image_filename)
            cv2.imwrite(image_save_path, cv_image)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
            return

        # Process Arm Joint Angles
        ordered_arm_angles = []
        for joint_name in self.arm_joint_names:
            if joint_name in self.latest_joint_states_msg.name:
                index = self.latest_joint_states_msg.name.index(joint_name)
                ordered_arm_angles.append(float(self.latest_joint_states_msg.position[index]))
            else:
                self.get_logger().warn(f'Joint {joint_name} not found in joint states')
                ordered_arm_angles.append(0.0)  # Default value if joint not found

        # Process Gripper Value
        gripper_int_value = 0
        if self.gripper_joint_name in self.latest_joint_states_msg.name:
            index = self.latest_joint_states_msg.name.index(self.gripper_joint_name)
            gripper_angle_rad = float(self.latest_joint_states_msg.position[index])

            # Scale and clip the gripper value
            normalized_value = (gripper_angle_rad - self.gripper_value_min_rad) / \
                              (self.gripper_value_max_rad - self.gripper_value_min_rad)
            normalized_value = max(0.0, min(1.0, normalized_value))  # Clip to [0, 1]

            gripper_int_value = int(normalized_value *
                                   (self.gripper_value_output_max_int - self.gripper_value_output_min_int) +
                                   self.gripper_value_output_min_int)
        else:
            self.get_logger().warn(f'Gripper joint {self.gripper_joint_name} not found in joint states')

        # Create JSON entry for this timestep
        json_entry = {
            "angles": ordered_arm_angles,
            "gripper_value": [gripper_int_value],
            "image": os.path.join(self.image_dir_name, image_filename)
        }

        self.current_episode_data_list.append(json_entry)
        self.frame_counter += 1

        if self.frame_counter % 10 == 0:
            self.get_logger().info(f'Logged frame {self.frame_counter} for episode {self.current_episode_id}')

    def handle_start_stop_service(self, request, response):
        """
        Handle requests to start or stop recording an episode.

        Args:
            request: The service request containing start_recording flag and episode_identifier
            response: The service response to be filled

        Returns:
            The filled service response
        """
        if request.start_recording:
            if self.is_recording:
                response.success = False
                response.message = "Already recording an episode"
                return response

            self.current_episode_id = request.episode_identifier
            self.current_episode_path = os.path.join(self.output_base_dir, self.current_episode_id)
            self.current_image_dir_path = os.path.join(self.current_episode_path, self.image_dir_name)

            # Create directories
            try:
                os.makedirs(self.current_image_dir_path, exist_ok=True)
                self.get_logger().info(f"Created episode directory: {self.current_episode_path}")
                self.get_logger().info(f"Created image directory: {self.current_image_dir_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to create directories: {str(e)}")
                response.success = False
                response.message = f"Failed to create directories: {str(e)}"
                return response

            self.current_episode_data_list = []
            self.frame_counter = 0
            self.is_recording = True

            response.success = True
            response.message = f"Started recording episode: {self.current_episode_id}"
            self.get_logger().info(response.message)
        else:
            if not self.is_recording:
                response.success = False
                response.message = "Not currently recording any episode"
                return response

            self.is_recording = False
            self.save_episode_json_file()

            response.success = True
            response.message = f"Stopped and saved episode: {self.current_episode_id}"
            self.get_logger().info(response.message)

            self.current_episode_id = None

        return response

    def save_episode_json_file(self):
        """Save the collected episode data to a JSON file."""
        if not self.current_episode_data_list or self.current_episode_path is None:
            self.get_logger().warn("No data to save or episode path not set")
            return

        json_file_path = os.path.join(self.current_episode_path, "states.json")
        self.get_logger().info(f"Attempting to save JSON data to: {json_file_path}")
        self.get_logger().info(f"Current episode path: {self.current_episode_path}")
        self.get_logger().info(f"Number of frames to save: {len(self.current_episode_data_list)}")

        try:
            # Make sure the directory exists
            os.makedirs(os.path.dirname(json_file_path), exist_ok=True)

            with open(json_file_path, 'w') as f:
                json.dump(self.current_episode_data_list, f, indent=2)

            self.get_logger().info(f"Saved JSON data to: {json_file_path}")
            self.get_logger().info(f"Total frames: {self.frame_counter}")
        except Exception as e:
            self.get_logger().error(f"Error saving JSON data: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    """Run the state logger node."""
    rclpy.init(args=args)

    state_logger_node = StateLoggerNode()

    # Use a multithreaded executor to handle callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(state_logger_node)

    try:
        state_logger_node.get_logger().info('Starting state logger node')
        executor.spin()
    except KeyboardInterrupt:
        state_logger_node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        state_logger_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
