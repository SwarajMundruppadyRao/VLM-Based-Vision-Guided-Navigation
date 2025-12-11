# Contributing to OCR-VLM-Based Vision-Guided Navigation

Thank you for your interest in contributing! This document provides guidelines for contributing to the project.

## How to Contribute

### Reporting Issues

If you find a bug or have a suggestion:

1. Check if the issue already exists in the issue tracker
2. If not, create a new issue with:
   - Clear title and description
   - Steps to reproduce (for bugs)
   - Expected vs. actual behavior
   - System information (OS, ROS version, etc.)

### Submitting Changes

1. **Fork the repository**
   ```bash
   # Fork on GitHub, then clone your fork
   git clone https://github.com/YOUR_USERNAME/VLM-Based-Vision-Guided-Navigation.git
   ```

2. **Create a feature branch**
   ```bash
   git checkout -b feature/your-feature-name
   ```

3. **Make your changes**
   - Write clean, readable code
   - Follow the existing code style
   - Add comments for complex logic
   - Update documentation as needed

4. **Test your changes**
   ```bash
   # Build the workspace
   colcon build
   
   # Test affected nodes
   ros2 launch vision_navigation_bringup navigation_system.launch.py
   ```

5. **Commit your changes**
   ```bash
   git add .
   git commit -m "Add feature: description of changes"
   ```

6. **Push to your fork**
   ```bash
   git push origin feature/your-feature-name
   ```

7. **Create a Pull Request**
   - Go to the original repository on GitHub
   - Click "New Pull Request"
   - Select your branch
   - Describe your changes clearly

## Code Style Guidelines

### Python

- Follow PEP 8 style guide
- Use meaningful variable names
- Add docstrings to functions and classes
- Keep functions focused and small

Example:
```python
def process_navigation_command(self, command):
    """
    Process navigation command and generate velocity.
    
    Args:
        command (str): Navigation command string
        
    Returns:
        Twist: Velocity command message
    """
    # Implementation here
    pass
```

### ROS 2 Conventions

- Use snake_case for topics: `/ocr/detected_text`
- Use PascalCase for message types: `NavigationCommand`
- Use snake_case for parameters: `confidence_threshold`
- Follow ROS 2 naming conventions

### Documentation

- Update README.md if adding features
- Document new parameters in config files
- Add examples for new functionality
- Update ARCHITECTURE.md for structural changes

## Development Setup

1. **Install dependencies**
   ```bash
   sudo apt-get install ros-humble-* python3-pip
   pip3 install -r requirements.txt
   ```

2. **Build workspace**
   ```bash
   colcon build
   source install/setup.bash
   ```

3. **Run tests** (when available)
   ```bash
   colcon test
   ```

## Areas for Contribution

We welcome contributions in these areas:

### High Priority

- [ ] Enhanced VLM model integration (BLIP-2, LLaVA, etc.)
- [ ] Comprehensive testing framework
- [ ] Performance optimization
- [ ] Better error handling and recovery

### Features

- [ ] Multi-camera support
- [ ] Path planning integration
- [ ] Obstacle avoidance
- [ ] SLAM integration
- [ ] Web-based monitoring interface
- [ ] Advanced OCR engines (EasyOCR, PaddleOCR)

### Documentation

- [ ] Tutorial videos
- [ ] More examples
- [ ] API documentation
- [ ] Troubleshooting guide

### Testing

- [ ] Unit tests for individual nodes
- [ ] Integration tests
- [ ] Simulation tests with Gazebo
- [ ] Performance benchmarks

## Pull Request Checklist

Before submitting a PR, ensure:

- [ ] Code builds without errors
- [ ] Code follows style guidelines
- [ ] Documentation is updated
- [ ] Commit messages are clear
- [ ] No unnecessary files included (build artifacts, etc.)
- [ ] Changes are focused (one feature per PR)

## Communication

- **Issues**: For bugs and feature requests
- **Pull Requests**: For code contributions
- **Discussions**: For questions and ideas

## Code Review Process

1. Maintainers review PRs within 1-2 weeks
2. Feedback is provided via PR comments
3. Address feedback and update PR
4. Once approved, PR is merged

## Recognition

Contributors will be:
- Listed in the project README
- Mentioned in release notes
- Credited in commit history

## Questions?

If you have questions about contributing:
1. Check existing documentation
2. Look at existing issues/PRs
3. Create a new issue with your question

Thank you for contributing to make robot navigation smarter! 🤖
