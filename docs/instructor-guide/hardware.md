---
title: Instructor Guide - Hardware Setup
sidebar_position: 22
---

# Instructor Guide - Hardware Setup

This section provides comprehensive guidance for setting up and maintaining the hardware infrastructure required for the Physical AI & Humanoid Robotics course. Proper hardware setup is essential for both instructor demonstrations and student lab experiences.

## Lab Station Requirements

### Minimum Specifications per Student Station
- **Computer**: Intel i7-10700K or AMD Ryzen 7 3700X
- **RAM**: 32GB DDR4-3200 MHz
- **GPU**: NVIDIA RTX 3070 (8GB VRAM) or better
- **Storage**: 500GB NVMe SSD
- **Network**: Gigabit Ethernet with WiFi 6 capability
- **OS**: Ubuntu 22.04 LTS preinstalled

### Recommended Specifications per Student Station
- **Computer**: Intel i9-12900K or AMD Ryzen 9 5900X
- **RAM**: 64GB DDR4-3200 MHz
- **GPU**: NVIDIA RTX 4080/4090 (16GB+ VRAM) or RTX A5000/A6000 for professional use
- **Storage**: 1TB NVMe SSD + 2TB HDD for additional data
- **Network**: Gigabit Ethernet with WiFi 6 capability
- **OS**: Ubuntu 22.04 LTS with ROS 2 Humble preinstalled

## Robot Platform Setup

### Universal Robot (UR3/UR5) Setup
1. **Initial Setup**:
   - Install URScript development environment
   - Configure network connection between workstation and robot
   - Set up safety configuration with appropriate safety zones

2. **Software Integration**:
   - Install ROS 2 UR driver packages
   - Configure MoveIt! planning for the specific robot model
   - Test ROS 2 communication with the physical robot

3. **Safety Protocols**:
   - Install physical safety barriers around robot workspace
   - Configure e-stop buttons accessible to all students
   - Install safety laser scanners if not already integrated

### Franka Emika Panda Setup
1. **Initial Setup**:
   - Calibrate the robot in its physical location
   - Install FCI (Franka Control Interface) with appropriate control modes
   - Configure collision detection parameters

2. **Software Integration**:
   - Install franka_ros2 packages
   - Calibrate camera-to-robot hand-eye coordination
   - Test impedance control modes for safe human-robot interaction

3. **Maintenance**:
   - Regular backup of robot calibration parameters
   - Check cable wear and replace as needed
   - Update robot firmware periodically following manufacturer guidelines

## Vision System Setup

### Intel RealSense D435/D435i
1. **Hardware Installation**:
   - Mount camera with appropriate field of view for workspace
   - Secure mounting to prevent vibrations that affect calibration
   - Connect to USB 3.0+ port with adequate power delivery

2. **Software Configuration**:
   - Install RealSense ROS 2 wrapper
   - Calibrate camera intrinsic and extrinsic parameters
   - Configure streaming parameters (resolution, FPS) based on application needs

3. **Troubleshooting**:
   - Check USB bandwidth saturation when using multiple cameras
   - Address IR interference in multi-camera setups
   - Handle temperature drift affecting calibration accuracy

### Alternative Vision Systems
Consider other options based on specific needs:
- **ZED Stereo Camera**: Better depth accuracy but higher computational requirements
- **Azure Kinect**: Good RGB and depth quality with additional IMU integration
- **Custom RGB-D**: For specialized applications requiring specific capabilities

## Network and Infrastructure

### Local Network Configuration
1. **Robot Communication**:
   - Dedicated subnet for robot communication to avoid network congestion
   - Static IP configuration for robot controllers
   - Quality of Service (QoS) configuration to prioritize robot communication

2. **Student Access**:
   - VLAN separation if needed for security and performance
   - Wireless access points with adequate coverage and bandwidth
   - Network isolation to prevent interference with other labs

### Cloud Integration (Optional)
For courses incorporating cloud robotics:
- VPN setup for secure cloud connectivity
- Cloud credentials management for student access
- Bandwidth planning for data-intensive operations

## Safety Equipment

### Mandatory Safety Items
- **Safety Goggles**: Appropriate for robotics lab environment
- **First Aid Kit**: Easily accessible and regularly checked
- **Fire Extinguisher**: Appropriate class for electrical equipment
- **Emergency Contacts**: Posted with local emergency services and robot manufacturer

### Recommended Safety Items
- **Safety Barriers**: Adjustable barriers for different robot workspace configurations
- **Signage**: Clear safety instructions and warnings
- **Personal Protective Equipment**: Lab coats, safety shoes if required by local regulations

## Maintenance Schedule

### Daily Checks (Before Each Lab Session)
- Verify robot homing and basic functionality
- Check network connectivity to all devices
- Ensure safety systems are operational
- Verify software licenses are available

### Weekly Maintenance
- Update software packages and security patches
- Check robot calibration and re-calibrate if necessary
- Clean vision system lenses and check mounting
- Back up important data and configurations

### Monthly Maintenance
- Comprehensive backup of all system configurations
- Check and replace worn components
- Review and update safety procedures
- Assess hardware performance for optimization opportunities

## Budget Considerations

### Cost-Saving Alternatives
- **Simulation-Only Option**: Begin with simulation before adding physical robots
- **Shared Resources**: Multiple courses sharing high-cost equipment
- **Gradual Rollout**: Phase in equipment over multiple semesters
- **Used Equipment**: Consider certified refurbished robots to reduce costs

### Cost Breakdown for Typical Lab (12 Stations)
- **Workstations (12)**: $15,000 - $36,000
- **Robots (6 shared units)**: $72,000 - $180,000
- **Vision Systems (12)**: $6,000 - $12,000
- **Networking Infrastructure**: $2,000 - $5,000
- **Safety Equipment**: $2,000 - $4,000
- **Total Initial Investment**: $97,000 - $237,000

## Troubleshooting Common Issues

### Network Connectivity Problems
- **Symptom**: Robot not responding to commands
- **Solution**: Check IP configuration, firewall settings, and cable connections
- **Prevention**: Document network configuration and create quick-diagnostic tools

### Vision System Calibration Drift
- **Symptom**: Decreasing accuracy in object detection or positioning
- **Solution**: Re-calibrate cameras and verify mounting stability
- **Prevention**: Regular calibration checks and vibration-dampening mounts

### Software Dependency Conflicts
- **Symptom**: ROS packages not building or running correctly
- **Solution**: Use Docker containers or virtual machines for isolated environments
- **Prevention**: Maintain documented, tested system images

## Student Preparation and Safety Training

### Pre-Lab Requirements
- Completion of safety training module
- Demonstration of basic Linux and programming skills
- Understanding of basic electrical safety

### Safety Protocols
1. **Robot Operation**:
   - Everyone maintains safe distance during robot motion
   - Immediate e-stop activation for any unsafe situation
   - Proper startup and shutdown procedures

2. **Equipment Handling**:
   - Proper lifting techniques for equipment
   - Safe cable management to prevent tripping
   - Appropriate attire for lab environment

## Setting Up Student Accounts

### ROS 2 Environment
- Pre-configure ROS 2 workspace with required packages
- Set up necessary environment variables
- Verify all required dependencies are installed and accessible

### Version Control and Collaboration
- Set up GitLab/GitHub classroom for assignment submission
- Configure access permissions and repository templates
- Implement backup strategies for student work

## Technical Support Resources

### Internal Support
- Designated technical support staff familiar with all systems
- Student assistant program with advanced students
- Regular maintenance schedule with documented procedures

### External Support
- Robot manufacturer support agreements
- Vision system technical support
- ROS community resources and forums

## Accessibility Accommodations

### For Students with Physical Disabilities
- Adjustable-height workstations
- Alternative input methods for computer interaction
- Clear pathways around robot workspaces
- Alternative assessment methods when appropriate

### For Students with Visual Impairments
- Audio feedback systems for robot status
- High-contrast displays and interfaces
- Alternative formats for visual materials
- Screen reader compatibility for all software

## Next Steps

After completing the hardware setup:

1. **Test All Systems**: Run comprehensive tests of all integrated systems
2. **Create Documentation**: Develop student reference guides for equipment use
3. **Safety Training**: Conduct safety training for all students and staff
4. **Backup Procedures**: Implement regular backup and recovery procedures

Continue to [Weekly Lesson Plans](#) or return to [Instructor Guide Home](./intro.md).

## Navigation

[← Previous: Instructor Guide Introduction](./intro.md) | [Next: Weekly Lesson Plans →](../ros2/week1-2.md) | [Instructor Guide Home →](./intro.md)