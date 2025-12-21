---
title: Instructor Guide - Troubleshooting Guide
sidebar_position: 26
---

# Instructor Guide - Troubleshooting Guide

This comprehensive troubleshooting guide addresses common technical and pedagogical issues that may arise during the Physical AI & Humanoid Robotics course. This resource enables instructors to quickly resolve problems and minimize disruption to learning.

## General Troubleshooting Approach

### Systematic Problem-Solving Steps
1. **Identify the Problem**: Clearly define what is not working
2. **Reproduce the Issue**: Verify the problem can be consistently reproduced
3. **Isolate Components**: Determine which system component is causing the issue
4. **Check Prerequisites**: Verify all dependencies and configurations are correct
5. **Research Solutions**: Consult documentation and community resources
6. **Implement Solution**: Apply the most appropriate fix
7. **Verify Resolution**: Confirm the problem is fully resolved
8. **Document Resolution**: Record the issue and solution for future reference

### Documentation Requirements
- Record all troubleshooting steps taken
- Note any system configurations changed
- Document the root cause of the problem
- Update this guide with new solutions discovered

## ROS 2 Troubleshooting

### Common Node Issues

#### Node Not Starting
**Symptoms**: 
- Node fails to start with no clear error message
- Node starts but doesn't appear in `ros2 node list`

**Diagnosis Steps**:
1. Check for syntax errors: `python3 -m py_compile your_node.py`
2. Verify ROS 2 environment: `echo $ROS_DISTRO`
3. Check for missing dependencies: `pip list | grep -i ros`
4. Run with increased verbosity: `python3 your_node.py --ros-args --log-level debug`

**Solutions**:
- Ensure workspace is sourced: `source install/setup.bash`
- Fix Python syntax or import errors
- Install missing package dependencies
- Check correct ROS 2 import statements

#### Topic Communication Failures
**Symptoms**:
- Publishers and subscribers not communicating
- Messages are not being received

**Diagnosis Steps**:
1. Check topic names: `ros2 topic list`
2. Verify data types: `ros2 topic info /your_topic`
3. Confirm QoS settings match between pub/sub
4. Check network configuration if using multi-machine setup

**Solutions**:
- Ensure topic names exactly match (including case)
- Verify message types are consistent
- Match QoS profiles between publisher and subscriber
- Check RMW implementation and network configuration

#### Service/Action Failures
**Symptoms**:
- Service calls hang or return errors
- Actions fail to complete as expected

**Diagnosis Steps**:
1. Check service availability: `ros2 service list`
2. Verify service type: `ros2 service type <service_name>`
3. Test with ros2 cli: `ros2 service call ...`

**Solutions**:
- Ensure service server is running before client calls
- Check service interface definitions match
- Implement proper error handling in service callbacks

### Performance Issues

#### High CPU Usage
**Symptoms**:
- ROS 2 nodes consuming excessive CPU resources
- System becomes slow or unresponsive

**Diagnosis Steps**:
1. Use `top` or `htop` to identify high CPU processes
2. Check for infinite loops in callbacks
3. Monitor node update rates with `ros2 topic hz`

**Solutions**:
- Add appropriate sleep statements in control loops
- Use rate limiters for message publishing
- Optimize algorithms for better computational efficiency

#### Memory Leaks
**Symptoms**:
- Memory usage increases over time
- System runs out of memory after extended operation

**Diagnosis Steps**:
1. Monitor memory usage with `free -h` or system monitor
2. Use Python memory profiler if using Python nodes
3. Check for unmanaged object references

**Solutions**:
- Implement proper object cleanup in callbacks
- Use weak references where appropriate
- Monitor and limit message queue sizes

## Simulation Environment Issues (Gazebo/Unity)

### Gazebo Troubleshooting

#### Graphics Rendering Problems
**Symptoms**:
- Gazebo interface appears black or distorted
- Textures not loading properly
- Slow rendering performance

**Diagnosis Steps**:
1. Check GPU compatibility and drivers: `nvidia-smi` or `lspci | grep -i display`
2. Verify OpenGL support: `glxinfo | grep -i opengl`
3. Test with software rendering: `export LIBGL_ALWAYS_SOFTWARE=1`

**Solutions**:
- Update GPU drivers to latest version
- Install appropriate graphics libraries: `sudo apt install mesa-utils`
- Adjust Gazebo rendering settings for performance
- Use software rendering for headless or virtual environments

#### Physics Simulation Issues
**Symptoms**:
- Objects behaving unrealistically
- Simulation running too fast or too slow
- Objects passing through each other

**Diagnosis Steps**:
1. Check world file physics parameters
2. Verify model SDF/URDF specifications
3. Monitor simulation real-time factor (RTF)

**Solutions**:
- Adjust physics engine parameters in world file
- Verify model collision and inertial properties
- Optimize model complexity for real-time performance

#### Plugin Loading Failures
**Symptoms**:
- Custom sensors or controllers not working
- Plugin-specific errors in console
- Missing functionality in simulation

**Diagnosis Steps**:
1. Check plugin library paths in model definitions
2. Verify plugin dependencies are installed
3. Confirm plugin syntax in SDF files

**Solutions**:
- Ensure correct library paths in plugin definitions
- Install required development packages
- Check plugin compatibility with Gazebo version

### Unity Troubleshooting

#### Import/Export Issues
**Symptoms**:
- ROS 2 Unity packages not importing correctly
- Missing references after import
- Build failures after ROS 2 integration

**Diagnosis Steps**:
1. Check Unity version compatibility with ROS 2 package
2. Verify required Unity packages are installed
3. Check for conflicting package versions

**Solutions**:
- Use compatible Unity and ROS 2 package versions
- Install required Unity packages via Package Manager
- Clear Library folder and reimport all assets

## NVIDIA Isaac Issues

### Perception Pipeline Problems

#### Detection Failures
**Symptoms**:
- Object detection not working
- Incorrect detection results
- High latency in detection pipeline

**Diagnosis Steps**:
1. Verify Isaac ROS packages installation: `ros2 pkg list | grep isaac`
2. Check camera calibration and mounting
3. Test image quality and lighting conditions

**Solutions**:
- Verify camera calibration using Isaac calibration tools
- Adjust detection thresholds and parameters
- Improve lighting conditions for better detection

#### SLAM System Issues
**Symptoms**:
- Map building failing or incorrect
- Robot localization drifting
- Loop closure failures

**Diagnosis Steps**:
1. Check sensor configurations and calibrations
2. Verify sufficient feature points in environment
3. Monitor tracking quality metrics

**Solutions**:
- Ensure adequate texture and features in environment
- Calibrate sensors properly before SLAM operation
- Adjust SLAM parameters based on environment characteristics

## Vision-Language-Action Model Issues

### Model Loading Problems
**Symptoms**:
- Large models failing to load
- CUDA out of memory errors
- Long initialization times

**Diagnosis Steps**:
1. Check GPU memory availability: `nvidia-smi`
2. Verify model file integrity
3. Confirm framework compatibility

**Solutions**:
- Use model quantization techniques to reduce memory usage
- Implement model loading in smaller chunks
- Use CPU fallback for memory-intensive operations

### Inference Performance
**Symptoms**:
- Slow response times for command interpretation
- Real-time performance requirements not met
- Model producing inconsistent results

**Diagnosis Steps**:
1. Profile inference times with timing tools
2. Check for bottlenecks in preprocessing
3. Verify batch size optimization

**Solutions**:
- Optimize preprocessing pipelines
- Adjust batch sizes for optimal throughput
- Use model optimization techniques (TensorRT, ONNX)

## Hardware Troubleshooting

### Robot Communication Issues

#### Network Connectivity
**Symptoms**:
- Robot not responding to commands
- Intermittent connection losses
- High communication latency

**Diagnosis Steps**:
1. Check network cables and connections
2. Test network latency: `ping robot_ip_address`
3. Verify firewall settings on both systems

**Solutions**:
- Use wired connections when possible
- Configure appropriate network QoS
- Isolate robot network from other traffic

#### Robot Control Problems
**Symptoms**:
- Robot not executing commands
- Trajectory deviations from planned path
- Unexpected movements or stops

**Diagnosis Steps**:
1. Check robot power and safety systems
2. Verify joint limits and collision checking
3. Monitor controller status and error codes

**Solutions**:
- Check safety system configurations
- Verify command parameters are within safe limits
- Update robot calibration if necessary

### Sensor Issues

#### Camera Problems
**Symptoms**:
- No image data or poor image quality
- Inconsistent frame rates
- Calibration failures

**Diagnosis Steps**:
1. Check camera power and USB connections
2. Verify driver installation: `lsusb`
3. Test camera with standard tools

**Solutions**:
- Reinstall camera drivers if necessary
- Check USB bandwidth limitations
- Recalibrate camera if mounting changed

#### Depth Sensor Issues
**Symptoms**:
- Inaccurate depth measurements
- Missing or noisy depth data
- Alignment problems between RGB and depth streams

**Diagnosis Steps**:
1. Check sensor calibration data
2. Verify adequate lighting for active sensors
3. Monitor data quality metrics

**Solutions**:
- Recalibrate sensors using appropriate tools
- Adjust exposure and gain settings
- Filter noisy data appropriately

## Software Environment Issues

### Python Environment Problems
**Symptoms**:
- Import errors for required packages
- Version conflicts between packages
- Virtual environment activation failures

**Diagnosis Steps**:
1. Check Python version: `python3 --version`
2. List installed packages: `pip3 list`
3. Verify virtual environment status: `echo $VIRTUAL_ENV`

**Solutions**:
- Use virtual environments to isolate dependencies
- Pin package versions in requirements.txt
- Use conda environments for complex dependency management

### Docker Container Issues
**Symptoms**:
- Containers failing to build or run
- GPU access problems in containers
- Network connectivity issues with containers

**Diagnosis Steps**:
1. Check Docker service status: `systemctl status docker`
2. Verify nvidia-docker installation: `nvidia-docker version`
3. Test basic container operation

**Solutions**:
- Install appropriate nvidia-container-toolkit
- Configure Docker daemon for GPU access
- Use appropriate base images for robotics applications

## Common Student Issues

### Initial Setup Problems
**Common Issues**:
- ROS 2 installation failures
- Workspace creation problems
- Environment variable configuration

**Quick Solutions**:
- Provide pre-configured VM or container images
- Create detailed installation scripts
- Use systematic checklist for setup verification

### Code Debugging Challenges
**Common Issues**:
- Understanding error messages
- Identifying runtime vs. compilation errors
- Debugging distributed system issues

**Teaching Strategies**:
- Use systematic debugging approaches
- Emphasize error message reading skills
- Practice debugging in controlled environments

## Performance Optimization

### System Performance Bottlenecks
**Identification**:
- Monitor CPU, memory, and disk usage
- Check network utilization
- Profile specific applications

**Optimization Strategies**:
- Adjust process priorities
- Optimize resource allocation
- Implement caching where appropriate

## Recovery Procedures

### System Restore Points
- Maintain system images before major changes
- Document current working configurations
- Create scripts for rapid system recovery

### Data Backup and Recovery
- Regular backup of important project data
- Version control for all code and configurations
- Cloud storage for critical system images

## Advanced Troubleshooting

### Diagnostic Tools
- ROS 2 command line tools: `ros2 doctor`, `ros2 bag`
- System monitoring: `htop`, `iotop`, `nethogs`
- Network tools: `netstat`, `nmap`, `wireshark`
- GPU monitoring: `nvidia-smi`, `nvtop`

### Log Analysis
- ROS 2 logging system configuration
- System log monitoring
- Application-specific logging
- Log aggregation and analysis tools

## Prevention Strategies

### Regular Maintenance
- System updates and security patches
- Hardware inspection and cleaning
- Software cleanup and optimization
- Backup verification

### Early Warning Systems
- System monitoring dashboards
- Automated health checks
- Performance baseline comparisons
- Alert systems for critical failures

## Support Resources

### Documentation
- Official ROS 2 documentation
- Hardware manufacturer documentation
- Troubleshooting guides for specific equipment
- Student-created troubleshooting wiki

### Community Resources
- ROS Discourse forums
- GitHub issue trackers for packages
- Robotics Stack Exchange
- Professional organizations and mailing lists

### Escalation Process
1. Attempt resolution using this guide
2. Consult with technical support staff
3. Search community resources and documentation
4. Contact manufacturer or package maintainers if needed

## Next Steps

Continue to [Course Evaluation Methods](#) or return to [Instructor Guide Home](./intro.md).

## Navigation

[← Previous: Pedagogical Notes](./pedagogical-notes.md) | [Next: Course Evaluation →](#) | [Instructor Guide Home →](./intro.md)