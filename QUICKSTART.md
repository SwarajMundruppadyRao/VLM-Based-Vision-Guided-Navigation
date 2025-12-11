# Quick Start Guide

This guide will help you get the OCR-VLM-Based Vision-Guided Navigation system up and running.

## Prerequisites Check

Before starting, ensure you have:

- [ ] Ubuntu 22.04 LTS
- [ ] ROS 2 Humble installed
- [ ] Python 3.8 or higher
- [ ] A camera connected (or use a simulated camera)

## Installation Steps

### 1. Install ROS 2 Humble (if not already installed)

Follow the official ROS 2 Humble installation guide:
https://docs.ros.org/en/humble/Installation.html

### 2. Install System Dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
    tesseract-ocr \
    python3-pip \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs
```

### 3. Clone and Build

```bash
# Clone the repository
git clone https://github.com/SwarajMundruppadyRao/VLM-Based-Vision-Guided-Navigation.git
cd VLM-Based-Vision-Guided-Navigation

# Install Python dependencies
pip3 install -r requirements.txt

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

### 4. Verify Installation

```bash
# List available packages
ros2 pkg list | grep vision

# Expected output:
# vision_control
# vision_navigation_bringup
# vision_navigation_msgs
# vision_ocr
# vision_vlm
```

## Running the System

### Option 1: Launch Complete System

```bash
source install/setup.bash
ros2 launch vision_navigation_bringup navigation_system.launch.py
```

This launches all three nodes (OCR, VLM, Control) simultaneously.

### Option 2: Launch Individual Nodes

**Terminal 1 - OCR Node:**
```bash
source install/setup.bash
ros2 launch vision_navigation_bringup ocr_only.launch.py
```

**Terminal 2 - VLM Node:**
```bash
source install/setup.bash
ros2 launch vision_navigation_bringup vlm_only.launch.py
```

**Terminal 3 - Control Node:**
```bash
source install/setup.bash
ros2 launch vision_navigation_bringup control_only.launch.py
```

## Testing Without a Camera

If you don't have a physical camera, you can test with simulated data:

### 1. Publish Test Image

```bash
# Install image publisher if needed
sudo apt-get install ros-humble-image-publisher

# Publish a test image (replace with your image path)
ros2 run image_publisher image_publisher_node /path/to/image.jpg
```

### 2. Publish Test Text Directly

```bash
# Simulate OCR output
ros2 topic pub /ocr/detected_text std_msgs/String "data: 'Turn Left'"
```

### 3. Monitor Commands

```bash
# Watch navigation commands
ros2 topic echo /vlm/navigation_command

# Watch velocity commands
ros2 topic echo /cmd_vel
```

## Checking Topic Communication

Verify that data flows through the system:

```bash
# List all active topics
ros2 topic list

# Check message rate on OCR output
ros2 topic hz /ocr/detected_text

# Check message rate on VLM output
ros2 topic hz /vlm/navigation_command

# Check message rate on control output
ros2 topic hz /cmd_vel
```

## Visualizing with RViz (Optional)

```bash
# Launch RViz for visualization
ros2 run rviz2 rviz2

# Add topics in RViz:
# - /camera/image_raw (Image)
# - /cmd_vel (Twist)
```

## Troubleshooting

### Issue: OCR node not detecting text

**Solution:**
- Check camera connection: `ros2 topic list | grep camera`
- Verify camera is publishing: `ros2 topic hz /camera/image_raw`
- Lower confidence threshold in `src/vision_ocr/config/ocr_params.yaml`

### Issue: No velocity commands published

**Solution:**
- Check that VLM commands are being published: `ros2 topic echo /vlm/navigation_command`
- Verify control node is running: `ros2 node list | grep control`
- Check command timeout hasn't expired

### Issue: Build failures

**Solution:**
```bash
# Clean build
rm -rf build install log

# Rebuild
colcon build --cmake-clean-cache
```

### Issue: Python dependencies missing

**Solution:**
```bash
# Reinstall dependencies
pip3 install -r requirements.txt --upgrade
```

## Next Steps

- Read [ARCHITECTURE.md](ARCHITECTURE.md) for system details
- Modify configuration files in `src/*/config/` directories
- Integrate with your robot platform
- Add custom navigation behaviors
- Extend VLM model capabilities

## Getting Help

If you encounter issues:
1. Check the logs: `cat log/latest_build/events.log`
2. Review node output for error messages
3. Open an issue on GitHub with details

## Quick Reference

| Command | Description |
|---------|-------------|
| `colcon build` | Build the workspace |
| `source install/setup.bash` | Source the workspace |
| `ros2 launch vision_navigation_bringup navigation_system.launch.py` | Launch full system |
| `ros2 node list` | List active nodes |
| `ros2 topic list` | List active topics |
| `ros2 topic echo <topic>` | Monitor topic messages |

Happy navigating! 🤖
