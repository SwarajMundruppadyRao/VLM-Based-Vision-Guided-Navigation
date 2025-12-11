# Frequently Asked Questions (FAQ)

## General Questions

### What is this project?

This is an OCR-VLM-Based Vision-Guided Navigation system for ROS 2 Humble. It enables robots to navigate autonomously by reading and interpreting text in their environment using Optical Character Recognition (OCR) and Vision-Language Models (VLM).

### What can the robot do?

The robot can:
- Detect text in its environment (signs, labels, room numbers)
- Understand the meaning of the text using AI
- Generate navigation commands (turn, move, stop, etc.)
- Execute those commands to navigate autonomously

### What hardware do I need?

Minimum requirements:
- A computer running Ubuntu 22.04
- ROS 2 Humble installed
- A camera (USB webcam, robot camera, etc.)
- Optional: A mobile robot platform

### Do I need a GPU?

No, the system works on CPU. However, a GPU will improve:
- VLM model inference speed
- Real-time performance
- Ability to use larger models

## Installation Questions

### How do I install ROS 2 Humble?

Follow the official guide: https://docs.ros.org/en/humble/Installation.html

Quick commands for Ubuntu 22.04:
```bash
sudo apt update
sudo apt install ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Why does the build fail?

Common causes:
1. Missing dependencies - run: `sudo apt-get install ros-humble-cv-bridge ros-humble-sensor-msgs`
2. Wrong ROS version - ensure you have ROS 2 Humble
3. Environment not sourced - run: `source /opt/ros/humble/setup.bash`

### How do I install Python dependencies?

```bash
pip3 install -r requirements.txt
```

If you get permission errors:
```bash
pip3 install -r requirements.txt --user
```

## Usage Questions

### How do I start the system?

```bash
source install/setup.bash
ros2 launch vision_navigation_bringup navigation_system.launch.py
```

### Can I test without a robot?

Yes! You can:
1. Test with simulated data using `examples/test_text_publisher.py`
2. Publish test images manually
3. Use the monitor script to see outputs

### How do I customize the behavior?

Edit the configuration files:
- OCR: `src/vision_ocr/config/ocr_params.yaml`
- VLM: `src/vision_vlm/config/vlm_params.yaml`
- Control: `src/vision_control/config/control_params.yaml`

### What text commands does it understand?

Default commands:
- "Stop", "Halt", "Wait" → STOP
- "Turn Left", "Left" → TURN_LEFT
- "Turn Right", "Right" → TURN_RIGHT
- "Forward", "Go", "Ahead" → MOVE_FORWARD
- "Back", "Reverse" → MOVE_BACKWARD
- "Exit", "Door" → MOVE_TO_EXIT
- "Room 123" → MOVE_TO_ROOM_123

You can add more in `src/vision_vlm/vision_vlm/vlm_node.py`

## Technical Questions

### What OCR engine is used?

Tesseract OCR. You can modify `ocr_node.py` to use alternatives like:
- EasyOCR
- PaddleOCR
- Google Cloud Vision
- AWS Textract

### What VLM models are supported?

Currently uses rule-based logic as a placeholder. To add VLM:

1. Install model libraries:
   ```bash
   pip3 install transformers torch
   ```

2. Modify `vlm_node.py` to load your chosen model:
   - BLIP-2
   - LLaVA
   - GPT-4 Vision
   - Gemini Vision

### How do I change robot speed?

Edit `src/vision_control/config/control_params.yaml`:
```yaml
control_node:
  ros__parameters:
    linear_speed: 0.2    # Change this (m/s)
    angular_speed: 0.5   # Change this (rad/s)
```

### How do I add a new navigation command?

1. Add keyword detection in `vlm_node.py`:
   ```python
   elif 'your_keyword' in text_lower:
       return 'YOUR_COMMAND'
   ```

2. Add command handling in `control_node.py`:
   ```python
   elif command == 'YOUR_COMMAND':
       twist.linear.x = 0.3
       twist.angular.z = 0.0
   ```

## Troubleshooting

### The OCR node doesn't detect any text

Try:
1. Check camera: `ros2 topic echo /camera/image_raw`
2. Lower confidence threshold in config
3. Ensure good lighting and text visibility
4. Test with clear, printed text

### The robot doesn't move

Check:
1. Is control node running? `ros2 node list`
2. Are commands being published? `ros2 topic echo /vlm/navigation_command`
3. Is your robot subscribed to `/cmd_vel`?
4. Check command timeout setting

### Nodes start but nothing happens

Verify topic connections:
```bash
ros2 topic list  # See all topics
ros2 topic hz /ocr/detected_text  # Check message rate
ros2 node info ocr_node  # Check node details
```

### Build errors with colcon

Try:
```bash
rm -rf build install log
colcon build --cmake-clean-cache
```

### Python import errors

Make sure workspace is sourced:
```bash
source install/setup.bash
```

And dependencies are installed:
```bash
pip3 install -r requirements.txt
```

## Performance Questions

### How fast does it process images?

Default: 2 Hz (configurable in `ocr_params.yaml`)

Factors affecting speed:
- Image resolution
- Preprocessing enabled/disabled
- CPU/GPU performance
- OCR complexity

### Can I increase the processing rate?

Yes, edit `processing_rate` in config, but consider:
- Higher CPU usage
- Potential lag on slower systems
- Diminishing returns above 5-10 Hz

### Is real-time navigation possible?

Yes, with:
- Modern CPU (i5 or better)
- Optimized parameters
- Good lighting conditions
- Clear, visible text

## Integration Questions

### How do I connect to my robot?

Your robot should subscribe to `/cmd_vel` (geometry_msgs/Twist). This is standard for most ROS robots.

### Can I use this with Gazebo?

Yes! Launch Gazebo with a simulated robot that:
1. Publishes camera images to `/camera/image_raw`
2. Subscribes to `/cmd_vel`

### Does it work with TurtleBot?

Yes! TurtleBot uses standard `/cmd_vel` topic. Just ensure camera topic matches.

### Can I integrate with Nav2?

Yes! You can:
- Use this as a high-level planner
- Publish goal poses instead of velocities
- Combine with Nav2 for full autonomy

## Development Questions

### How do I add a new node?

1. Create new package: `ros2 pkg create --build-type ament_python my_package`
2. Add node in `my_package/my_package/my_node.py`
3. Update `setup.py` entry points
4. Add to bringup launch file

### How do I add custom messages?

Add to `vision_navigation_msgs/msg/` and rebuild.

### Where should I implement SLAM?

Create a new package `vision_slam` and integrate with VLM node for scene understanding.

### How do I contribute?

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines!

## Still Have Questions?

- Check [ARCHITECTURE.md](ARCHITECTURE.md) for system details
- See [QUICKSTART.md](QUICKSTART.md) for getting started
- Review [examples/README.md](examples/README.md) for testing
- Open an issue on GitHub for specific problems

## Useful Commands Reference

```bash
# Build workspace
colcon build

# Source workspace
source install/setup.bash

# Launch system
ros2 launch vision_navigation_bringup navigation_system.launch.py

# List nodes
ros2 node list

# List topics
ros2 topic list

# Echo topic
ros2 topic echo /ocr/detected_text

# Publish to topic
ros2 topic pub /ocr/detected_text std_msgs/String "data: 'Stop'"

# Check message rate
ros2 topic hz /cmd_vel

# Node info
ros2 node info ocr_node

# Visualize graph
rqt_graph
```
