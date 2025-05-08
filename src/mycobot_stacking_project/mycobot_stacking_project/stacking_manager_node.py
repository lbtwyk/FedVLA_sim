#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import threading
import sys

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.msg import CollisionObject
from moveit_py import MoveItPy, PlanningSceneMonitor
from tf_transformations import quaternion_from_euler


class StackingManagerNode(Node):

    # Constants for configuration
    ARM_GROUP = "arm"
    GRIPPER_GROUP = "gripper"
    EEF_LINK = "link6_flange" # Check SRDF for actual EEF link
    GRIPPER_OPEN_STATE = "open" # Check SRDF for actual name
    GRIPPER_CLOSE_STATE = "close" # Check SRDF for actual name
    HOME_STATE = "home" # Check SRDF for actual name

    YELLOW_CUBE_ID = "yellow_cube"
    ORANGE_CUBE_ID = "orange_cube"
    CUBE_SIZE = 0.025

    APPROACH_DISTANCE = 0.10 # Z-offset for pre-grasp/pre-place
    LIFT_DISTANCE = 0.08     # Z-offset for lifting after grasp

    # Planning parameters
    PLANNING_TIME_S = 5.0
    PLANNING_ATTEMPTS = 5

    def __init__(self):
        super().__init__('stacking_manager')
        self.get_logger().info('Initializing Stacking Manager Node...')

        # Initialize MoveItPy
        # IMPORTANT: MoveItPy needs to be spun in a separate thread.
        # This is essential for it to handle callbacks, services, etc.
        self.moveit_py = MoveItPy(node=self)
        self.get_logger().info('MoveItPy created.')

        # Use a lock for thread safety when accessing MoveIt components
        self.moveit_lock = threading.Lock()

        # Spin MoveItPy in a separate thread
        self.moveit_thread = threading.Thread(target=self._spin_moveit, daemon=True)
        self.moveit_thread.start()
        self.get_logger().info('MoveItPy spin thread started.')

        # Wait briefly for MoveIt components to initialize
        # A more robust approach might involve checking service availability
        time.sleep(3.0) # Increased sleep time

        # Get planning components - access within the lock
        with self.moveit_lock:
            try:
                self.arm = self.moveit_py.get_planning_component(self.ARM_GROUP)
                if self.arm is None:
                     self.get_logger().error(f"Could not get planning component for group '{self.ARM_GROUP}'. Exiting.")
                     sys.exit(1)
                self.get_logger().info(f'Got planning component for group: {self.ARM_GROUP}')

                self.gripper = self.moveit_py.get_planning_component(self.GRIPPER_GROUP)
                if self.gripper is None:
                     self.get_logger().error(f"Could not get planning component for group '{self.GRIPPER_GROUP}'. Exiting.")
                     sys.exit(1)
                self.get_logger().info(f'Got planning component for group: {self.GRIPPER_GROUP}')

                # Get Planning Scene Monitor
                self.planning_scene_monitor = self.moveit_py.get_planning_scene_monitor()
                if self.planning_scene_monitor is None:
                    self.get_logger().error("Could not get planning scene monitor. Exiting.")
                    sys.exit(1)
                self.get_logger().info('Got planning scene monitor.')

            except Exception as e:
                self.get_logger().error(f"Failed to initialize MoveIt components: {e}. Exiting.")
                sys.exit(1)

        self.get_logger().info('Stacking Manager Node Initialized Successfully.')

    def _spin_moveit(self):
        """ Function to spin rclpy for the MoveItPy node """
        self.get_logger().info('Starting MoveItPy spin thread...')
        try:
            # Make sure rclpy is initialized before spinning
            if rclpy.ok():
                 # Create a single-threaded executor for the MoveItPy node
                executor = rclpy.executors.SingleThreadedExecutor()
                executor.add_node(self.moveit_py)
                executor.spin()
            else:
                 self.get_logger().warn('RCLpy not OK in MoveItPy spin thread.')
        except Exception as e:
            self.get_logger().error(f'Exception in MoveItPy spin thread: {e}')
        finally:
             self.get_logger().info('MoveItPy spin thread finished.')
             # We don't shut down rclpy here as the main thread might still need it.

    def wait_for_cubes(self, timeout_sec=30.0):
        """Waits until both cubes appear in the planning scene."""
        start_time = self.get_clock().now()
        self.get_logger().info(f"Waiting for cubes '{self.YELLOW_CUBE_ID}' and '{self.ORANGE_CUBE_ID}'...")

        while rclpy.ok():
            current_time = self.get_clock().now()
            if (current_time - start_time).nanoseconds / 1e9 > timeout_sec:
                self.get_logger().error(f'Timeout waiting for cubes after {timeout_sec} seconds.')
                return None, None

            yellow_pose = None
            orange_pose = None
            found_yellow = False
            found_orange = False

            try:
                 with self.planning_scene_monitor.read_only() as scene:
                    if not scene:
                        self.get_logger().warn("Planning scene is not available yet.")
                        time.sleep(0.5)
                        continue

                    # Check if collision_objects is populated
                    if not hasattr(scene.world, 'collision_objects') or not scene.world.collision_objects:
                        self.get_logger().debug("Planning scene world has no collision objects yet.")
                    else:
                        collision_objects_dict = {co.id: co for co in scene.world.collision_objects}
                        # self.get_logger().info(f"Current objects: {list(collision_objects_dict.keys())}")

                        if self.YELLOW_CUBE_ID in collision_objects_dict:
                            co = collision_objects_dict[self.YELLOW_CUBE_ID]
                            # Check if primitive_poses is not empty before accessing
                            if co.primitive_poses:
                                yellow_pose = co.primitive_poses[0] # Assuming box primitive is first
                                found_yellow = True
                                self.get_logger().debug(f'Found yellow cube at {yellow_pose.position}')
                            else:
                                self.get_logger().warn(f"Yellow cube '{self.YELLOW_CUBE_ID}' found but has no primitive_poses.")

                        if self.ORANGE_CUBE_ID in collision_objects_dict:
                            co = collision_objects_dict[self.ORANGE_CUBE_ID]
                            if co.primitive_poses:
                                orange_pose = co.primitive_poses[0]
                                found_orange = True
                                self.get_logger().debug(f'Found orange cube at {orange_pose.position}')
                            else:
                                 self.get_logger().warn(f"Orange cube '{self.ORANGE_CUBE_ID}' found but has no primitive_poses.")

                    if found_yellow and found_orange:
                        self.get_logger().info('Found both cubes!')
                        # Ensure the poses are relative to base_link (perception node should handle this)
                        # Add frame check if necessary
                        return yellow_pose, orange_pose

            except Exception as e:
                self.get_logger().error(f"Error accessing planning scene: {e}")

            time.sleep(1.0) # Check periodically
        return None, None # Return None if rclpy is shut down

    def move_to_pose(self, planning_component, target_pose_stamped: PoseStamped, operation_desc="Move to Pose"):
        """Plans and executes a motion to a target pose."""
        self.get_logger().info(f'{operation_desc}: Planning to pose: {target_pose_stamped.pose.position}')
        with self.moveit_lock:
            if not self.arm:
                 self.get_logger().error("Arm planning component not initialized.")
                 return False
            self.arm.set_goal_state(pose_stamped_msg=target_pose_stamped)
            plan_solution = self.arm.plan()

        if plan_solution and plan_solution.trajectory:
            self.get_logger().info(f'{operation_desc}: Plan found, executing...')
            with self.moveit_lock:
                if not self.moveit_py:
                     self.get_logger().error("MoveItPy not initialized for execution.")
                     return False
                moveit_controller_result = self.moveit_py.execute(plan_solution.trajectory, controllers=[])

            if moveit_controller_result.execution_successful:
                self.get_logger().info(f'{operation_desc}: Execution successful.')
                return True
            else:
                self.get_logger().error(f'{operation_desc}: Execution failed: {moveit_controller_result.error_code.val}')
                return False
        else:
            self.get_logger().error(f'{operation_desc}: Planning failed.')
            return False

    def move_to_named_state(self, planning_component, state_name, operation_desc="Move to Named State"):
        """Plans and executes a motion to a named state."""
        self.get_logger().info(f"{operation_desc}: Planning to state '{state_name}'...")
        with self.moveit_lock:
            if not planning_component:
                 self.get_logger().error(f"Planning component for {operation_desc} not initialized.")
                 return False
            planning_component.set_goal_state(configuration_name=state_name)
            plan_solution = planning_component.plan()

        if plan_solution and plan_solution.trajectory:
            self.get_logger().info(f'{operation_desc}: Plan found, executing...')
            with self.moveit_lock:
                 if not self.moveit_py:
                     self.get_logger().error("MoveItPy not initialized for execution.")
                     return False
                 moveit_controller_result = self.moveit_py.execute(plan_solution.trajectory, controllers=[])

            if moveit_controller_result.execution_successful:
                self.get_logger().info(f'{operation_desc}: Execution successful.')
                return True
            else:
                self.get_logger().error(f'{operation_desc}: Execution failed: {moveit_controller_result.error_code.val}')
                return False
        else:
            self.get_logger().error(f'{operation_desc}: Planning failed.')
            return False

    def move_linearly(self, planning_component, target_pose: Pose, operation_desc="Move Linearly"):
        """ Plans and executes a linear Cartesian motion. """
        self.get_logger().info(f'{operation_desc}: Planning linear move...')
        with self.moveit_lock:
            if not self.arm:
                 self.get_logger().error("Arm planning component not initialized.")
                 return False
            # Compute Cartesian path
            (plan, fraction) = self.arm.compute_cartesian_path(
                [target_pose], # List of waypoints
                0.01, # eef_step - resolution of 1 cm
                0.0, # jump_threshold - disable jumps
                avoid_collisions=True,
            )

        if plan and fraction == 1.0:
            self.get_logger().info(f'{operation_desc}: Cartesian path found (fraction: {fraction}), executing...')
            with self.moveit_lock:
                if not self.moveit_py:
                     self.get_logger().error("MoveItPy not initialized for execution.")
                     return False
                moveit_controller_result = self.moveit_py.execute(plan, controllers=[])

            if moveit_controller_result.execution_successful:
                self.get_logger().info(f'{operation_desc}: Linear execution successful.')
                return True
            else:
                self.get_logger().error(f'{operation_desc}: Linear execution failed: {moveit_controller_result.error_code.val}')
                return False
        else:
            self.get_logger().error(f'{operation_desc}: Cartesian path planning failed (fraction: {fraction}). Cannot execute linear move.')
            return False

    def move_gripper(self, state_name):
        """Moves the gripper to a predefined named state ('open' or 'close')."""
        return self.move_to_named_state(self.gripper, state_name, operation_desc=f"Move Gripper to {state_name}")

    def attach_object(self, object_id):
        """Attaches the specified object to the end effector."""
        self.get_logger().info(f"Attaching object '{object_id}' to EEF '{self.EEF_LINK}'")
        try:
            with self.planning_scene_monitor.read_write() as scene:
                if not scene:
                    self.get_logger().error("Planning scene not available for attaching object.")
                    return False
                collision_object = CollisionObject()
                collision_object.id = object_id
                collision_object.header.frame_id = self.EEF_LINK # Attach relative to EEF
                collision_object.operation = CollisionObject.ADD # Operation is ADD to attached object

                scene.attach_object(collision_object, self.EEF_LINK, []) # Empty touch_links for now
                self.get_logger().info("Attach request sent.")
            # Allow some time for the planning scene to update
            time.sleep(0.5)
            return True
        except Exception as e:
            self.get_logger().error(f"Error attaching object: {e}")
            return False


    def detach_object(self, object_id):
        """Detaches the specified object from the end effector."""
        self.get_logger().info(f"Detaching object '{object_id}' from EEF '{self.EEF_LINK}'")
        try:
            with self.planning_scene_monitor.read_write() as scene:
                if not scene:
                    self.get_logger().error("Planning scene not available for detaching object.")
                    return False
                collision_object = CollisionObject()
                collision_object.id = object_id
                collision_object.operation = CollisionObject.REMOVE # Remove from attached objects

                scene.detach_object(collision_object)
                self.get_logger().info("Detach request sent.")

                 # Add the object back to the world scene after detaching
                scene.add_object(collision_object) # This might need pose adjustment

            # Allow some time for the planning scene to update
            time.sleep(0.5)
            return True
        except Exception as e:
            self.get_logger().error(f"Error detaching object: {e}")
            return False

    def run_stacking_task(self):
        self.get_logger().info('Starting cube stacking task...')

        # 1. Wait for cubes to be detected
        yellow_cube_pose, orange_cube_pose = self.wait_for_cubes()
        if not yellow_cube_pose or not orange_cube_pose:
            self.get_logger().error('Failed to detect cubes. Aborting task.')
            return

        # --- Define Target Poses relative to base_link ---
        # (Assuming perception node provides poses in base_link)
        base_frame = "base_link" # Expect poses here

        # TODO: Verify cube poses are in base_frame

        # Grasping pose orientation (pointing downwards)
        # Example: Rotate -180 degrees around Y axis (roll=0, pitch=-pi, yaw=0)
        grasp_orientation = Quaternion(
             *quaternion_from_euler(0.0, -3.14159, 0.0)
        )

        # a. Yellow Pre-Grasp Pose
        yellow_pre_grasp_pose_stamped = PoseStamped()
        yellow_pre_grasp_pose_stamped.header.frame_id = base_frame
        yellow_pre_grasp_pose_stamped.pose = Pose(
            position=Point(
                x=yellow_cube_pose.position.x,
                y=yellow_cube_pose.position.y,
                z=yellow_cube_pose.position.z + self.APPROACH_DISTANCE
            ),
            orientation=grasp_orientation
        )

        # b. Yellow Grasp Pose (adjust Z for gripper finger height)
        yellow_grasp_pose = Pose(
            position=Point(
                x=yellow_cube_pose.position.x,
                y=yellow_cube_pose.position.y,
                # Target center of cube slightly above bottom
                z=yellow_cube_pose.position.z + self.CUBE_SIZE * 0.5 + 0.005 # Small offset
            ),
            orientation=grasp_orientation
        )

        # c. Lift Pose
        lift_pose = Pose(
            position=Point(
                x=yellow_grasp_pose.position.x,
                y=yellow_grasp_pose.position.y,
                z=yellow_grasp_pose.position.z + self.LIFT_DISTANCE
            ),
            orientation=grasp_orientation
        )

        # d. Orange Pre-Place Pose
        orange_pre_place_pose_stamped = PoseStamped()
        orange_pre_place_pose_stamped.header.frame_id = base_frame
        orange_pre_place_pose_stamped.pose = Pose(
            position=Point(
                x=orange_cube_pose.position.x,
                y=orange_cube_pose.position.y,
                z=orange_cube_pose.position.z + self.CUBE_SIZE + self.APPROACH_DISTANCE
            ),
            orientation=grasp_orientation
        )

        # e. Orange Place Pose
        orange_place_pose = Pose(
            position=Point(
                x=orange_cube_pose.position.x,
                y=orange_cube_pose.position.y,
                z=orange_cube_pose.position.z + self.CUBE_SIZE + 0.01 # Place slightly above orange cube
            ),
            orientation=grasp_orientation
        )

        # --- Execute Sequence ---
        success = True

        # Go home
        success &= self.move_to_named_state(self.arm, self.HOME_STATE, "Go Home")
        if not success: return self.get_logger().error("Failed to go home.")

        # Open gripper
        success &= self.move_gripper(self.GRIPPER_OPEN_STATE)
        if not success: return self.get_logger().error("Failed to open gripper.")

        # Move to yellow pre-grasp
        success &= self.move_to_pose(self.arm, yellow_pre_grasp_pose_stamped, "Move to Yellow Pre-Grasp")
        if not success: return self.get_logger().error("Failed to move to yellow pre-grasp.")

        # Approach yellow linearly
        success &= self.move_linearly(self.arm, yellow_grasp_pose, "Approach Yellow Cube")
        if not success: return self.get_logger().error("Failed to approach yellow cube.")

        # Close gripper (Grasp)
        success &= self.move_gripper(self.GRIPPER_CLOSE_STATE)
        if not success: return self.get_logger().error("Failed to close gripper.")
        time.sleep(0.5) # Allow gripper to close

        # Attach yellow cube
        success &= self.attach_object(self.YELLOW_CUBE_ID)
        if not success: return self.get_logger().error("Failed to attach yellow cube.")

        # Lift yellow cube linearly
        success &= self.move_linearly(self.arm, lift_pose, "Lift Yellow Cube")
        if not success: return self.get_logger().error("Failed to lift yellow cube.")

        # Move to orange pre-place
        success &= self.move_to_pose(self.arm, orange_pre_place_pose_stamped, "Move to Orange Pre-Place")
        if not success: return self.get_logger().error("Failed to move to orange pre-place.")

        # Lower yellow cube linearly
        success &= self.move_linearly(self.arm, orange_place_pose, "Lower Yellow onto Orange")
        if not success: return self.get_logger().error("Failed to lower yellow cube.")

        # Open gripper (Release)
        success &= self.move_gripper(self.GRIPPER_OPEN_STATE)
        if not success: return self.get_logger().error("Failed to open gripper for release.")
        time.sleep(0.5) # Allow gripper to open

        # Detach yellow cube
        success &= self.detach_object(self.YELLOW_CUBE_ID)
        if not success: return self.get_logger().error("Failed to detach yellow cube.")

        # Retreat linearly
        # Create PoseStamped for retreat target
        retreat_pose_stamped = PoseStamped()
        retreat_pose_stamped.header.frame_id = base_frame
        retreat_pose_stamped.pose = orange_pre_place_pose_stamped.pose # Use pre-place pose for retreat
        # Using move_to_pose for retreat as linear might be restrictive here
        success &= self.move_to_pose(self.arm, retreat_pose_stamped, "Retreat from Stack")
        # success &= self.move_linearly(self.arm, orange_pre_place_pose_stamped.pose, "Retreat from Stack")
        if not success: return self.get_logger().error("Failed to retreat from stack.")

        # Go home
        success &= self.move_to_named_state(self.arm, self.HOME_STATE, "Go Home After Stack")
        if not success: return self.get_logger().error("Failed to go home after stacking.")

        if success:
            self.get_logger().info('Cube stacking task completed successfully!')
        else:
            self.get_logger().error('Cube stacking task failed at some point.')


def main(args=None):
    rclpy.init(args=args)
    node = StackingManagerNode()
    # Need to wait until node is fully initialized, especially MoveItPy
    # Adding a sleep here, but a more robust check might be needed.
    time.sleep(5.0)

    # Run the main task logic
    # Consider running this in a separate thread if the node needs to do other things
    try:
        if rclpy.ok(): # Check if init was successful and RCLpy is running
            node.run_stacking_task()
    except Exception as e:
         node.get_logger().fatal(f"Unhandled exception in main execution: {e}")
         import traceback
         traceback.print_exc()

    # Keep the node alive to keep the MoveItPy thread spinning if needed,
    # or implement logic to shutdown gracefully after the task.
    # For a single-shot task, we might not need to spin here after run_stacking_task finishes.
    # rclpy.spin(node) # If the node needs to stay alive for other reasons

    # Clean shutdown
    node.get_logger().info("Shutting down Stacking Manager Node.")
    # No need to explicitly stop the daemon thread
    node.destroy_node()
    # rclpy.shutdown() # Shutdown should ideally happen outside if multiple nodes run

if __name__ == '__main__':
    main()