---
title: Instructor Guide - Weekly Lesson Plans
sidebar_position: 23
---

# Instructor Guide - Weekly Lesson Plans

This section provides detailed lesson plans for each week of the Physical AI & Humanoid Robotics course, including learning objectives, activities, assessments, and pedagogical notes.

## General Weekly Structure

Each week typically follows this structure:
- **Day 1**: Theory and concepts introduction
- **Day 2**: Implementation and hands-on practice
- **Day 3**: Lab exercises and project work
- **Day 4**: Assessment and troubleshooting
- **Day 5**: Project work and advanced topics

## Week 1-2: ROS 2 Architecture and Concepts

### Learning Objectives
- Students will understand the fundamental architecture of ROS 2
- Students will create and run basic ROS 2 nodes in Python
- Students will implement message passing between nodes using topics and services

### Day 1: ROS 2 Architecture Overview
**Duration**: 2 hours lecture + 1 hour lab prep

**Activities**:
1. **Lecture** (60 min):
   - ROS 2 vs. ROS 1 architectural differences
   - DDS (Data Distribution Service) concept
   - Nodes, topics, services, actions explanation
   - Launch files and parameter management

2. **Lab Setup** (30 min):
   - Verify ROS 2 installation on student workstations
   - Basic ROS 2 commands demonstration
   - Workspace setup for the week

3. **Concept Check** (30 min):
   - Quick quiz on ROS 2 fundamental concepts
   - Discussion of common misconceptions

**Resources Needed**:
- Projector for demonstrations
- Prepared ROS 2 workspace with example packages
- Student handout with essential ROS 2 commands

**Instructor Notes**:
- Emphasize the distributed architecture concept early
- Address common confusion between topics and services
- Prepare troubleshooting guide for common installation issues

### Day 2: Creating ROS 2 Nodes
**Duration**: 1 hour lecture + 3 hours lab

**Activities**:
1. **Lecture** (30 min):
   - Node structure and lifecycle
   - Publisher and subscriber implementation
   - Parameter servers and configuration

2. **Hands-on Practice** (2.5 hours):
   - Students create their first ROS 2 node
   - Implement publisher and subscriber nodes
   - Debug common node creation issues

**Lab Exercise**:
- Create a simple talker/listener node pair
- Extend to broadcast sensor data from a simulated sensor

**Assessment**:
- Formative: Instructor observation during lab
- Students demonstrate successful node communication

**Instructor Notes**:
- Be prepared for Python version compatibility issues
- Have pre-built example nodes available for reference
- Monitor for common mistakes in node creation patterns

### Day 3: Services and Actions
**Duration**: 1 hour lecture + 3 hours lab

**Activities**:
1. **Lecture** (30 min):
   - Services vs. topics: when to use each
   - Actions for long-running tasks
   - Error handling and node resilience

2. **Hands-on Practice** (2.5 hours):
   - Implement service client and server
   - Create action client and server
   - Integrate services with existing nodes

**Lab Exercise**:
- Add a service to control robot movement
- Implement an action for path planning

**Assessment**:
- Students demonstrate working service and action implementations
- Peer review of code quality and error handling

**Instructor Notes**:
- Clarify the difference between services and actions early and often
- Prepare examples of real-world use cases for each
- Monitor for issues with asynchronous service calls

### Day 4: Launch Files and Parameter Management
**Duration**: 1 hour lecture + 2 hours lab

**Activities**:
1. **Lecture** (30 min):
   - Launch file syntax and structure
   - Parameter management best practices
   - Node composition and optimization

2. **Hands-on Practice** (1.5 hours):
   - Create launch files for multi-node systems
   - Implement parameter configuration
   - Optimize node performance

**Assessment**:
- Quiz on launch file syntax and parameter management
- Demonstration of properly configured multi-node system

**Instructor Notes**:
- Emphasize importance of parameter management in complex systems
- Prepare examples of well-structured launch files
- Be ready to debug complex launch file configurations

### Day 5: Week Review and Troubleshooting
**Duration**: 2 hours

**Activities**:
1. **Project Work** (1 hour):
   - Students work on integrated ROS 2 system
   - Instructors provide individualized support

2. **Troubleshooting Session** (1 hour):
   - Common problem patterns review
   - Debugging techniques demonstration
   - Q&A session

**Assessment**:
- Students present their integrated ROS 2 system
- Peer evaluation of system design and implementation

## Week 3: Advanced ROS 2 Concepts and Integration

### Learning Objectives
- Students will integrate multiple ROS 2 concepts into a cohesive system
- Students will implement error handling and system resilience
- Students will optimize ROS 2 systems for performance

### Day 1: System Integration Patterns
**Duration**: 2 hours lecture + 1 hour lab prep

**Activities**:
1. **Lecture** (60 min):
   - Design patterns for ROS 2 systems
   - Component architecture and reusability
   - Integration testing strategies

2. **Lab Preparation** (30 min):
   - Prepare integrated project workspace
   - Review integration goals and milestones

### Day 2-3: Project Implementation
**Duration**: 2x 4-hour lab sessions

**Activities**:
- Students work on comprehensive ROS 2 project
- Instructors provide guidance and feedback
- Daily checkpoints and progress reviews

### Day 4: Performance Optimization
**Duration**: 1 hour lecture + 2 hours lab

**Activities**:
- Performance profiling techniques
- Optimization strategies for real-time systems
- Memory and computation efficiency

### Day 5: Week Review and Assessment
**Duration**: 3 hours project presentations

## Week 4-5: Gazebo Simulation Environments

### Learning Objectives
- Students will create simulation environments in Gazebo
- Students will implement robot models and sensors in simulation
- Students will validate simulation against real-world data

### Day 1: Gazebo Basics and Environment Creation
**Duration**: 2 hours lecture + 1 hour lab

**Activities**:
1. **Lecture** (60 min):
   - Gazebo architecture and physics engine
   - World file creation and customization
   - Sensor integration and configuration

2. **Hands-on Practice** (60 min):
   - Create basic simulation environment
   - Add simple objects and terrain
   - Configure basic lighting and physics

**Instructor Notes**:
- Prepare sample world files for students to modify
- Check GPU compatibility for rendering-intensive simulations
- Have simplified environments ready for slower systems

### Day 2: Robot Models and Physics
**Duration**: 1 hour lecture + 3 hours lab

**Activities**:
1. **Lecture** (30 min):
   - URDF and SDF robot description formats
   - Physics properties and constraints
   - Collision and visual models

2. **Hands-on Practice** (2.5 hours):
   - Model a simple robot in URDF
   - Import into Gazebo environment
   - Test physical interactions

## Week 6: Sensors and Physics in Simulation

### Learning Objectives
- Students will integrate various sensor types in simulation
- Students will implement realistic physics models
- Students will validate simulation accuracy

## Week 7-8: Perception and VSLAM with NVIDIA Isaac

### Learning Objectives
- Students will implement perception algorithms using Isaac tools
- Students will create VSLAM systems for robot navigation
- Students will integrate perception with control systems

## Week 9: AI-Robot Brain Integration

### Learning Objectives
- Students will connect AI models to robot control systems
- Students will implement decision-making algorithms
- Students will optimize AI inference for robotic applications

## Week 10-11: Vision-Language Integration

### Learning Objectives
- Students will connect vision models with language understanding
- Students will implement multimodal fusion techniques
- Students will create language-conditional perception systems

## Week 12: Action Planning with LLMs

### Learning Objectives
- Students will integrate LLMs with robotic action planning
- Students will implement grounded language understanding
- Students will evaluate language-to-action systems

## Week 13: Course Synthesis and Capstone Project

### Learning Objectives
- Students will integrate all course modules into a single system
- Students will demonstrate comprehensive embodied AI system
- Students will evaluate and present their final projects

## Assessment Strategies

### Formative Assessment Tools
- Concept checks during lectures
- Peer programming exercises
- Real-time debugging challenges
- Code review sessions

### Summative Assessment Strategies
- Weekly lab reports with technical analysis
- Programming assignments with rubric-based grading
- Project presentations with demonstration
- Comprehensive final project evaluation

## Differentiation Strategies

### For Advanced Students
- Research paper implementations
- Extension projects beyond required scope
- Leadership roles in group activities
- Mentorship of classmates

### For Students Needing Additional Support
- Additional scaffolding and guided practice
- Pair programming with stronger students
- Extended lab time and additional office hours
- Simplified project requirements with clear milestones

## Common Student Misconceptions

### ROS 2-Specific Misconceptions
- Confusing asynchronous vs. synchronous communication
- Misunderstanding node lifecycle and error handling
- Confusing topics, services, and actions usage scenarios

### Simulation-Specific Misconceptions
- Assuming simulation perfectly matches reality
- Neglecting computational complexity in simulation
- Overlooking physics parameter significance

### AI Integration Misconceptions
- Expecting AI models to work without training/data
- Underestimating computational requirements
- Confusing perception with cognition

## Accessibility Accommodations

### For Students with Different Learning Needs
- Multiple modalities for content delivery
- Extended time for complex implementations
- Alternative assessment methods when needed
- Collaborative learning opportunities

## Technical Troubleshooting

### Preemptive Measures
- System checks before each lab session
- Backup hardware and software configurations
- Step-by-step troubleshooting guides
- Peer support systems

### Common Technical Issues
- Network connectivity problems
- Hardware calibration drift
- Software dependency conflicts
- Performance bottlenecks

## Extension Activities

### For Extra Credit
- Advanced research paper implementations
- Open source contribution projects
- Cross-course integration projects
- Conference-style presentations

## Next Steps

Continue to [Assessment Rubrics](#) or return to [Instructor Guide Home](./intro.md).

## Navigation

[← Previous: Hardware Setup](./hardware.md) | [Next: Assessment Rubrics →](#) | [Instructor Guide Home →](./intro.md)