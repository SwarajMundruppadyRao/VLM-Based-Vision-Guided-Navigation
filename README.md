# OCR-VLM-Based Vision-Guided Navigation (Ongoing❗)

This repository contains a ROS 2 Humble workspace implementing a vision-guided navigation system that integrates Optical Character Recognition (OCR), Vision-Language Models (VLM), and robot control capabilities. The system enables a robot to perceive text in its environment, interpret it using language models, and respond with appropriate navigation actions.

## Overview

The system consists of four main components:

1. **OCR Module**: Detects and extracts text from camera images using OCR technology
2. **VLM Module**: Processes visual and textual information using Vision-Language Models to understand context and generate navigation decisions
3. **Robot Control Module**: Executes navigation commands based on VLM outputs
4. **Integration Module**: Orchestrates communication between all components

## Architecture

```
Camera Images → OCR Node → Text Detection
                    ↓
              VLM Node → Language Understanding → Navigation Commands
                    ↓
         Robot Control Node → Motor Commands → Robot Movement
```

## Prerequisites

- ROS 2 Humble
- Python 3.8+
- OpenCV
- PyTesseract (for OCR)
- Transformers library (for VLM)
- PyTorch

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/SwarajMundruppadyRao/VLM-Based-Vision-Guided-Navigation.git
cd VLM-Based-Vision-Guided-Navigation
```

### 2. Install system dependencies

```bash
sudo apt-get update
sudo apt-get install -y tesseract-ocr python3-pip ros-humble-cv-bridge ros-humble-sensor-msgs
```

### 3. Install Python dependencies

```bash
pip3 install -r requirements.txt
```

### 4. Build the workspace

```bash
colcon build
source install/setup.bash
```

## Usage

### Launch the complete system

```bash
ros2 launch vision_navigation_bringup navigation_system.launch.py
```

### Launch individual nodes

**OCR Node:**
```bash
ros2 run vision_ocr ocr_node
```

**VLM Node:**
```bash
ros2 run vision_vlm vlm_node
```

**Control Node:**
```bash
ros2 run vision_control control_node
```

## Package Structure

```
src/
├── vision_ocr/              # OCR text detection package
├── vision_vlm/              # Vision-Language Model package
├── vision_control/          # Robot control package
├── vision_navigation_msgs/  # Custom message definitions
└── vision_navigation_bringup/  # Launch files and configuration
```

## ROS 2 Topics

- `/camera/image_raw` - Input camera images
- `/ocr/detected_text` - Detected text from OCR
- `/vlm/navigation_command` - Navigation commands from VLM
- `/cmd_vel` - Velocity commands to robot

## Configuration

Configuration files are located in the `config/` directory of each package. You can modify parameters such as:

- OCR confidence threshold
- VLM model selection
- Robot velocity limits
- Camera settings

## Development Status

This project is currently under development (Ongoing❗). Features being implemented:

- [x] ROS 2 workspace structure
- [x] OCR integration
- [x] VLM integration
- [x] Robot control interface
- [ ] Complete testing and validation
- [ ] Performance optimization
- [ ] Additional VLM models support

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## License

This project is open source and available under the MIT License.

## Acknowledgments

- ROS 2 Community
- OpenCV and Tesseract OCR projects
- Hugging Face Transformers library