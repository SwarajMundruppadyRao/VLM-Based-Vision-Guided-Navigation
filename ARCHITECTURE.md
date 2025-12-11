# Vision-Guided Navigation Architecture

## System Overview

The OCR-VLM-Based Vision-Guided Navigation system is designed to enable autonomous robot navigation using visual and textual environmental cues. The system processes camera images to detect text, interprets the text using vision-language models, and generates appropriate navigation commands.

## Component Architecture

### 1. OCR Module (`vision_ocr`)

**Purpose**: Detect and extract text from camera images

**Key Features**:
- Real-time text detection using Tesseract OCR
- Configurable confidence threshold filtering
- Image preprocessing for improved accuracy
- Publishes detected text to ROS 2 topics

**Topics**:
- Subscribes to: `/camera/image_raw` (sensor_msgs/Image)
- Publishes to: `/ocr/detected_text` (std_msgs/String)

### 2. VLM Module (`vision_vlm`)

**Purpose**: Process visual and textual information to generate navigation decisions

**Key Features**:
- Integration with Vision-Language Models
- Context-aware command generation
- Rule-based fallback system
- Multi-modal input processing

**Topics**:
- Subscribes to: `/ocr/detected_text` (std_msgs/String)
- Subscribes to: `/camera/image_raw` (sensor_msgs/Image)
- Publishes to: `/vlm/navigation_command` (std_msgs/String)

### 3. Control Module (`vision_control`)

**Purpose**: Execute navigation commands by controlling robot motion

**Key Features**:
- Command-to-velocity translation
- Safety timeouts
- Configurable speed limits
- Support for multiple command types

**Topics**:
- Subscribes to: `/vlm/navigation_command` (std_msgs/String)
- Publishes to: `/cmd_vel` (geometry_msgs/Twist)

### 4. Message Definitions (`vision_navigation_msgs`)

Custom ROS 2 messages and services:
- `DetectedText.msg`: Enhanced text detection with metadata
- `NavigationCommand.msg`: Structured navigation commands
- `ProcessScene.srv`: Service for on-demand scene processing

### 5. Bringup Package (`vision_navigation_bringup`)

Launch files and configurations for system deployment:
- `navigation_system.launch.py`: Launch all components
- Individual node launch files for testing
- Centralized configuration management

## Data Flow

```
Camera → Image → OCR Node → Detected Text → VLM Node → Navigation Command → Control Node → Robot Motion
                              ↓
                          VLM Node (also receives images for context)
```

## Command Types

The system supports the following navigation commands:

- `STOP`: Halt all motion
- `MOVE_FORWARD`: Move forward at configured speed
- `MOVE_BACKWARD`: Move backward at configured speed
- `TURN_LEFT`: Rotate left in place
- `TURN_RIGHT`: Rotate right in place
- `MOVE_TO_EXIT`: Navigate toward detected exit
- `MOVE_TO_ROOM_X`: Navigate to specific room
- `ANALYZE`: Pause for additional analysis

## Configuration

Each module has configurable parameters in YAML files:

### OCR Configuration
- `confidence_threshold`: Minimum confidence for text detection
- `processing_rate`: Image processing frequency (Hz)
- `preprocess_image`: Enable/disable image preprocessing

### VLM Configuration
- `model_name`: VLM model identifier
- `device`: CPU or GPU processing
- `max_response_length`: Maximum command length

### Control Configuration
- `linear_speed`: Forward/backward speed (m/s)
- `angular_speed`: Rotation speed (rad/s)
- `command_timeout`: Safety timeout (seconds)

## Extension Points

The system is designed for easy extension:

1. **OCR Engines**: Swap Tesseract for other OCR implementations
2. **VLM Models**: Add support for different vision-language models
3. **Control Strategies**: Implement advanced motion planning
4. **Custom Messages**: Define domain-specific message types
5. **Safety Layers**: Add obstacle avoidance and collision detection

## Testing

Individual components can be tested independently:

```bash
# Test OCR with a camera
ros2 run vision_ocr ocr_node

# Test VLM with simulated text
ros2 topic pub /ocr/detected_text std_msgs/String "data: 'Turn Left'"

# Test control with simulated commands
ros2 topic pub /vlm/navigation_command std_msgs/String "data: 'TURN_LEFT'"
```

## Performance Considerations

- OCR processing rate affects system latency
- VLM model size impacts memory and computation
- Command timeout prevents stuck states
- Image preprocessing improves accuracy but adds latency

## Future Enhancements

- Multi-camera support
- Advanced VLM integration (BLIP, LLaVA, etc.)
- Path planning and mapping
- Dynamic obstacle avoidance
- Learning-based navigation policies
