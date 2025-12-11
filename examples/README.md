# Example Scripts

This directory contains example scripts for testing and demonstrating the vision-guided navigation system.

## Available Scripts

### 1. test_text_publisher.py

A test script that publishes sample text messages to simulate OCR output.

**Usage:**
```bash
# Make sure the workspace is sourced
source install/setup.bash

# Run the test publisher
python3 examples/test_text_publisher.py
```

**What it does:**
- Publishes a sequence of test text commands to `/ocr/detected_text`
- Waits 3 seconds between each command
- Useful for testing VLM and Control nodes without a camera

**Test sequence:**
1. "Go Forward"
2. "Turn Left"
3. "Stop"
4. "Turn Right"
5. "Exit"
6. "Room 101"
7. "Move Ahead"

### 2. monitor_system.py

A monitoring script that displays all system activity in real-time.

**Usage:**
```bash
# Make sure the workspace is sourced
source install/setup.bash

# Run the monitor
python3 examples/monitor_system.py
```

**What it does:**
- Subscribes to `/ocr/detected_text`, `/vlm/navigation_command`, and `/cmd_vel`
- Displays all messages in a human-readable format
- Useful for debugging and understanding system behavior

**Output example:**
```
[OCR] Detected: "Turn Left"
[VLM] Command: TURN_LEFT
[CTRL] Motion: TURN_LEFT (w=0.50)
```

## Running a Complete Test

### Test Scenario 1: Full System Test

**Terminal 1 - Launch System:**
```bash
source install/setup.bash
ros2 launch vision_navigation_bringup navigation_system.launch.py
```

**Terminal 2 - Monitor Output:**
```bash
source install/setup.bash
python3 examples/monitor_system.py
```

**Terminal 3 - Send Test Commands:**
```bash
source install/setup.bash
python3 examples/test_text_publisher.py
```

### Test Scenario 2: Manual Testing

Send individual commands manually:

```bash
# Terminal 1 - Launch system
ros2 launch vision_navigation_bringup navigation_system.launch.py

# Terminal 2 - Send manual text
ros2 topic pub /ocr/detected_text std_msgs/String "data: 'Stop'"

# Terminal 3 - Monitor velocity
ros2 topic echo /cmd_vel
```

## Creating Custom Tests

You can create your own test scripts by following these patterns:

### Publishing Text (Python):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ... create node and publisher ...
msg = String()
msg.data = "Your text here"
publisher.publish(msg)
```

### Publishing Text (Command Line):
```bash
ros2 topic pub /ocr/detected_text std_msgs/String "data: 'Your text here'"
```

### Monitoring Topics:
```bash
# Echo a topic
ros2 topic echo /vlm/navigation_command

# Check message rate
ros2 topic hz /cmd_vel

# Show topic info
ros2 topic info /ocr/detected_text
```

## Troubleshooting Examples

If the examples don't work:

1. **Check ROS 2 environment:**
   ```bash
   echo $ROS_DISTRO  # Should show 'humble'
   ```

2. **Verify workspace is sourced:**
   ```bash
   source install/setup.bash
   ```

3. **Check if nodes are running:**
   ```bash
   ros2 node list
   ```

4. **Verify topics exist:**
   ```bash
   ros2 topic list
   ```

## Tips

- Make scripts executable: `chmod +x examples/*.py`
- Use `Ctrl+C` to stop running scripts
- Check logs if something doesn't work: `ros2 node info <node_name>`
- Use RQt to visualize topic flow: `rqt_graph`
