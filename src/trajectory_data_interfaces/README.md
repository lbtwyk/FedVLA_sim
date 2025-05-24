# Trajectory Data Interfaces

A ROS 2 package providing custom message and service definitions for the trajectory data collection system. This package defines the communication interfaces between data collection nodes and robot control systems.

## Overview

This package contains ROS 2 interface definitions for:

- **Service Interfaces**: Control data collection episodes
- **Message Types**: Custom data structures for trajectory recording
- **Communication Protocols**: Standardized interfaces for robot demonstration collection

## Package Structure

```
src/trajectory_data_interfaces/
├── CMakeLists.txt                   # Build configuration
├── package.xml                      # Package metadata
├── srv/                             # Service definitions
│   └── StartStopEpisode.srv         # Episode control service
├── msg/                             # Message definitions (future expansion)
└── include/                         # Generated headers (auto-generated)
```

## Service Definitions

### StartStopEpisode.srv

Controls the start and stop of data collection episodes.

**File**: `srv/StartStopEpisode.srv`

```
# Request
string command        # "start" or "stop"
string episode_id     # Unique identifier for the episode

---

# Response
bool success          # True if command was successful
string message        # Status message or error description
string episode_path   # Full path to episode directory (for start command)
```

**Usage**:

```bash
# Start a new episode
ros2 service call /start_stop_episode trajectory_data_interfaces/srv/StartStopEpisode \
  "{command: 'start', episode_id: 'episode_001'}"

# Stop current episode
ros2 service call /start_stop_episode trajectory_data_interfaces/srv/StartStopEpisode \
  "{command: 'stop', episode_id: ''}"
```

**Response Examples**:

Start Success:
```yaml
success: true
message: "Episode started successfully"
episode_path: "/home/user/mycobot_episodes/episode_20250524_143022_123"
```

Start Failure:
```yaml
success: false
message: "Episode already in progress"
episode_path: ""
```

Stop Success:
```yaml
success: true
message: "Episode stopped successfully. Recorded 150 frames."
episode_path: "/home/user/mycobot_episodes/episode_20250524_143022_123"
```

## Integration Examples

### Python Client (Robot Control Node)

```python
import rclpy
from rclpy.node import Node
from trajectory_data_interfaces.srv import StartStopEpisode
from datetime import datetime

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        # Create service client
        self.episode_client = self.create_client(
            StartStopEpisode, 
            '/start_stop_episode'
        )
        
        # Wait for service to be available
        while not self.episode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for episode service...')
    
    def start_episode(self, episode_id=None):
        """Start a new data collection episode."""
        if episode_id is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            episode_id = f"episode_{timestamp}"
        
        request = StartStopEpisode.Request()
        request.command = 'start'
        request.episode_id = episode_id
        
        future = self.episode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info(f"Started episode: {episode_id}")
            return future.result().episode_path
        else:
            self.get_logger().error(f"Failed to start episode: {future.result().message}")
            return None
    
    def stop_episode(self):
        """Stop the current data collection episode."""
        request = StartStopEpisode.Request()
        request.command = 'stop'
        request.episode_id = ''
        
        future = self.episode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info("Episode stopped successfully")
            return future.result().episode_path
        else:
            self.get_logger().error(f"Failed to stop episode: {future.result().message}")
            return None
```

### C++ Client (Robot Control Node)

```cpp
#include <rclcpp/rclcpp.hpp>
#include <trajectory_data_interfaces/srv/start_stop_episode.hpp>

class RobotControlNode : public rclcpp::Node
{
public:
    RobotControlNode() : Node("robot_control_node")
    {
        // Create service client
        episode_client_ = this->create_client<trajectory_data_interfaces::srv::StartStopEpisode>(
            "/start_stop_episode");
        
        // Wait for service
        while (!episode_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for episode service...");
        }
    }
    
    bool start_episode(const std::string& episode_id)
    {
        auto request = std::make_shared<trajectory_data_interfaces::srv::StartStopEpisode::Request>();
        request->command = "start";
        request->episode_id = episode_id;
        
        auto future = episode_client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Started episode: %s", episode_id.c_str());
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to start episode: %s", 
                           response->message.c_str());
                return false;
            }
        }
        
        RCLCPP_ERROR(this->get_logger(), "Service call failed");
        return false;
    }
    
    bool stop_episode()
    {
        auto request = std::make_shared<trajectory_data_interfaces::srv::StartStopEpisode::Request>();
        request->command = "stop";
        request->episode_id = "";
        
        auto future = episode_client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Episode stopped successfully");
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to stop episode: %s", 
                           response->message.c_str());
                return false;
            }
        }
        
        RCLCPP_ERROR(this->get_logger(), "Service call failed");
        return false;
    }

private:
    rclcpp::Client<trajectory_data_interfaces::srv::StartStopEpisode>::SharedPtr episode_client_;
};
```

### Python Service Server (Data Collection Node)

```python
import rclpy
from rclpy.node import Node
from trajectory_data_interfaces.srv import StartStopEpisode
import os
from datetime import datetime

class DataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')
        
        # Create service server
        self.episode_service = self.create_service(
            StartStopEpisode,
            '/start_stop_episode',
            self.handle_episode_request
        )
        
        self.current_episode = None
        self.recording = False
    
    def handle_episode_request(self, request, response):
        """Handle start/stop episode requests."""
        
        if request.command == 'start':
            if self.recording:
                response.success = False
                response.message = "Episode already in progress"
                response.episode_path = ""
            else:
                # Generate episode path
                if request.episode_id:
                    episode_id = request.episode_id
                else:
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                    episode_id = f"episode_{timestamp}"
                
                episode_path = os.path.join(self.output_base_dir, episode_id)
                
                # Start recording
                self.start_recording(episode_path)
                
                response.success = True
                response.message = "Episode started successfully"
                response.episode_path = episode_path
                
        elif request.command == 'stop':
            if not self.recording:
                response.success = False
                response.message = "No episode in progress"
                response.episode_path = ""
            else:
                # Stop recording
                frame_count = self.stop_recording()
                
                response.success = True
                response.message = f"Episode stopped successfully. Recorded {frame_count} frames."
                response.episode_path = self.current_episode
                
        else:
            response.success = False
            response.message = f"Unknown command: {request.command}"
            response.episode_path = ""
        
        return response
```

## Build and Installation

### Dependencies

Add to your `package.xml`:

```xml
<depend>rosidl_default_generators</depend>
<depend>rosidl_default_runtime</depend>
<depend>std_msgs</depend>
```

### CMakeLists.txt Configuration

```cmake
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartStopEpisode.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
```

### Building

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select trajectory_data_interfaces

# Source the workspace
source install/setup.bash
```

## Testing

### Service Availability

```bash
# Check if service is available
ros2 service list | grep start_stop_episode

# Get service type
ros2 service type /start_stop_episode
```

### Manual Testing

```bash
# Test start command
ros2 service call /start_stop_episode trajectory_data_interfaces/srv/StartStopEpisode \
  "{command: 'start', episode_id: 'test_episode'}"

# Test stop command  
ros2 service call /start_stop_episode trajectory_data_interfaces/srv/StartStopEpisode \
  "{command: 'stop', episode_id: ''}"

# Test invalid command
ros2 service call /start_stop_episode trajectory_data_interfaces/srv/StartStopEpisode \
  "{command: 'invalid', episode_id: ''}"
```

## Future Extensions

### Planned Message Types

- `EpisodeStatus.msg`: Real-time episode status information
- `TrajectoryPoint.msg`: Individual trajectory point data
- `EpisodeMetadata.msg`: Episode metadata and statistics

### Additional Services

- `GetEpisodeList.srv`: Retrieve list of available episodes
- `ValidateEpisode.srv`: Validate episode data integrity
- `DeleteEpisode.srv`: Remove episode data

## Dependencies

- ROS 2 Jazzy Jalisco
- rosidl_default_generators
- rosidl_default_runtime
- std_msgs

## Related Packages

- **[trajectory_data_collector](../trajectory_data_collector/README.md)** - Data collection implementation
- **[mycobot_stacking_project](../mycobot_stacking_project/README.md)** - Robot control integration

## Back to Main Documentation

← [Main Project README](../../README.md)
