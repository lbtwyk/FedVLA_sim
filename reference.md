It seems like you've made significant progress, and Gazebo and RViz are now launching, and the `move_group` node appears to be initializing correctly! However, the main issues now are related to **controller spawning failures** and the **cubes not appearing in RViz**.

Let's break down the errors from your output:

**Identified Errors & Issues:**

1.  **Controller Spawner Failures (Critical):**
    * `[ros2-9] [WARN] [1746550192.412158334] [_ros2cli_2598766]: Failed getting a result from calling /controller_manager/list_controllers in 10.0. (Attempt 1 of 3.)`
    * `[ros2-9] Failed loading controller joint_state_broadcaster check controller_manager logs`
    * `[ERROR] [ros2-9]: process has died [pid 2598766, exit code 1, cmd 'ros2 control load_controller --set-state active joint_state_broadcaster'].`
    * Similar errors for `arm_controller` and `gripper_action_controller`. For `gripper_action_controller`, it even says `already loaded, skipping load_controller!` but then `Error configuring controller`.
    * **Reason:** These errors clearly indicate that the `controller_manager` service `/controller_manager/list_controllers` (which is provided by the `gz_ros2_control` plugin running inside Gazebo) was not available when the `ros2 control load_controller ...` (spawner) commands were executed. This is a **sequencing issue**. The spawners are being called before the Controller Manager (within Gazebo, started by the plugin) is fully initialized and has advertised its services.

2.  **RViz: Robot Model Loading / Kinematics Warnings:**
    * `[rviz2-7] [ERROR] [1746550177.740691550] [rviz2.moveit.ros.background_processing]: Exception caught while processing action 'loadRobotModel': Invalid node name: node name must not contain characters other than alphanumerics or '_': 'rviz2_ssp_/robot_description'` (This repeats).
        * **Reason:** RViz's MoveIt plugin is trying to create a temporary internal node to load the robot description, but the generated name `rviz2_ssp_/robot_description` is invalid because of the `/`. This might be an internal MoveIt/RViz issue or a misconfiguration in how `robot_description` is passed to RViz's MoveIt display.
    * `[rviz2-7] [WARN] [1746550177.847776234] [rviz2.moveit.ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!`
        * **Reason:** RViz's instance of the `RobotModelLoader` (used by the MotionPlanning display) didn't find or load the `kinematics.yaml` file. This is strange because `move_group` itself seems to load kinematics correctly (`[move_group-6] [INFO] [1746550171.007752450] [move_group.moveit.moveit.kinematics.kdl_kinematics_plugin]: Joint weights for group 'arm': 1 1 1 1 1 1`). It suggests that RViz is not getting the complete MoveIt configuration that `move_group` has.

3.  **RViz: Cubes Not Appearing:**
    * You stated: "in gazebo the cubes are seen but in rviz the cube are not".
    * **Reason:** This means your `cube_detector_node` is likely either not running, not detecting the cubes correctly, not publishing them to the `/monitored_planning_scene` (or `/planning_scene`) topic, or RViz's PlanningScene display isn't configured to show them (though you checked "Show Scene Geometry" earlier).
        * The `cube_detector_node` initializes: `[cube_detector_node-8] [INFO] [1746550171.751942056] [cube_detector]: Cube Detector Node initialized and ready`.
        * The `stacking_manager_node` is waiting for the cubes: `[stacking_manager_node-10] [INFO] [1746550184.957247957] [stacking_manager]: Waiting for cubes 'yellow_cube' and 'orange_cube'...` This implies the perception node hasn't successfully added them to the planning scene in a way the `stacking_manager` (and RViz) can see them.

4.  **Publisher Already Registered Warnings (Minor, but indicate potential issues):**
    * `[cube_detector_node-8] [WARN] [1746550171.086874992] [rcl.logging_rosout]: Publisher already registered for node name: 'cube_detector'. ...`
    * Similar warnings for `rviz2` and `stacking_manager`.
    * **Reason:** This usually happens if nodes with the same name are launched multiple times, or if a node creates a publisher (like the rosout publisher for logs) and then another part of its initialization or a re-launch attempts to register it again. It can also happen if the node's logger object is not a singleton or is being re-initialized. While often benign for logging, it can indicate a deeper structural issue if nodes are indeed being launched multiple times. Given other nodes seem to start once, this might be related to how `rclpy`/`rclcpp` loggers are handled internally upon recreation of certain objects or within specific launch contexts.

5.  **Gazebo `<gz_frame_id>` Warnings (Likely Benign for functionality):**
    * `[gazebo-2] Warning [Utils.cc:132] [/sdf/model[@name="mycobot_280"]/link[@name="base_link"]/sensor[@name="camera_head"]/gz_frame_id:<data-string>:L186]: XML Element[gz_frame_id], child of element[sensor], not defined in SDF...`
    * **Reason:** The URDF/XACRO for your robot's camera sensor is using a `<gz_frame_id>` tag inside the `<sensor>` tag, which SDF might not strictly define there. Gazebo is just warning it's copying it as a child. This is common and usually doesn't break functionality if the frame ID is still correctly interpreted by the camera plugin and TF.

6.  **Gazebo `libEGL` Permission Denied (Minor, WSL2 specific):**
    * `[gazebo-2] libEGL warning: failed to open /dev/dri/renderD128: Permission denied`
    * **Reason:** Common in WSL2 when accessing GPU hardware for rendering if permissions aren't perfectly set up. Gazebo usually falls back to software rendering or a different path. If Gazebo GUI is appearing and seems to render okay, this might not be critical for simulation logic.

7.  **MoveIt Octomap Warnings (Can be ignored for now):**
    * `[move_group-6] [WARN] [1746550171.099607730] [move_group.moveit.moveit.ros.occupancy_map_monitor]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead`
    * `[move_group-6] [ERROR] [1746550171.099678171] [move_group.moveit.moveit.ros.occupancy_map_monitor]: No 3D sensor plugin(s) defined for octomap updates`
    * **Reason:** You haven't configured a 3D sensor (like your RealSense) to feed data directly into MoveIt for OctoMap generation. This is fine for now as your `cube_detector_node` is handling perception separately.

**Key Focus Areas for Fixing:**

1.  **Controller Spawner Sequencing (Highest Priority):** This is critical. Without active controllers, `move_group` cannot command the robot, and your `stacking_manager` will be stuck.
2.  **RViz Robot Model & Kinematics:** The errors in RViz's MoveIt display loading the robot model and kinematics need to be addressed for proper visualization and interactive planning in RViz.
3.  **Perception to Planning Scene:** Ensure `cube_detector_node` is correctly publishing collision objects and that `stacking_manager_node` and RViz are subscribing to the correct planning scene topics.

The Gazebo GUI and `move_group` itself seem to be starting up much better than before, which is great progress!