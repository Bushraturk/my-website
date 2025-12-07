---
title: Week 12 - Action Planning with Large Language Models
sidebar_position: 18
---

# Week 12: Action Planning with Large Language Models

In this week, we'll explore how large language models (LLMs) can be integrated with robotic systems to generate executable action plans from high-level natural language commands. You'll learn to connect LLMs with perception systems and low-level controllers to create intelligent robots that can follow complex instructions.

## Learning Objectives

By the end of this week, you will be able to:

- Integrate large language models with robotic systems for task planning
- Generate executable robot actions from natural language commands
- Implement grounded language understanding for robotic tasks
- Create language-conditioned action policies
- Evaluate the success of language-to-action translation in robotics

## Introduction to Language-Guided Action Planning

Large language models have shown remarkable ability to understand and generate human language, but connecting them to robotic action execution requires careful design considerations. The challenge lies in translating abstract linguistic commands into concrete robotic actions that are appropriate for the current environmental context.

### The Language-to-Action Pipeline

The language-to-action pipeline involves several key steps:

1. **Command Interpretation**: Parse natural language commands to extract action verbs, objects, and spatial relationships
2. **Context Understanding**: Combine language commands with environmental context from perception systems
3. **Action Planning**: Generate a sequence of low-level actions to achieve the commanded task
4. **Execution Monitoring**: Continuously monitor execution and adapt to environmental changes

### Approaches to Language-Guided Action

There are several approaches to implementing language-guided action:

- **Symbolic Planning**: Translate language commands to formal symbolic plans using classical planners
- **Neural Planning**: Use neural networks to learn direct mappings from language to actions
- **Reactive Approaches**: Implement rule-based systems that react to language input with predefined action sequences
- **Hybrid Methods**: Combine multiple approaches for greater flexibility and robustness

## Implementing LLM-Robot Integration

Let's implement a system that integrates large language models with robotic action execution:

```python
import openai
import json
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
from transformers import AutoTokenizer, AutoModelForCausalLM
from typing import Dict, List, Optional, Tuple


class LanguageActionPlanner:
    """
    A system that translates natural language commands into executable robot actions.
    Uses LLMs for command interpretation and action generation.
    """
    def __init__(self, llm_model_name: str = "gpt-3.5-turbo", robot_description: str = ""):
        """
        Initialize the language-action planner.
        
        Args:
            llm_model_name: Name of the LLM to use for command interpretation
            robot_description: Description of the robot's capabilities and current state
        """
        self.llm_model_name = llm_model_name
        self.robot_description = robot_description
        self.cv_bridge = CvBridge()
        
        # Robot state information
        self.current_state = {
            "location": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "gripper": "open",  # or "closed"
            "objects_in_workspace": []  # List of objects currently in workspace
        }
        
        # Available robot actions
        self.action_space = [
            "move_to_location(x, y)",
            "rotate_to_angle(theta)",
            "open_gripper()",
            "close_gripper()",
            "pick_up_object(object_name)",
            "place_object(object_name, location)",
            "navigate_to(location_name)",
            "look_at(location)",
            "detect_objects()"
        ]

    def interpret_command(self, command: str) -> Dict:
        """
        Use LLM to interpret a natural language command and generate an action plan.
        
        Args:
            command: Natural language command from the user
            
        Returns:
            Dictionary containing action plan and relevant parameters
        """
        # Construct prompt for the LLM
        prompt = f"""
        You are an intelligent robot assistant that translates human commands into robot actions.
        The robot has the following capabilities: {self.robot_description}
        
        Available actions are:
        {', '.join(self.action_space)}
        
        The robot's current state is:
        - Location: {self.current_state['location']}
        - Gripper: {self.current_state['gripper']}
        - Objects in workspace: {self.current_state['objects_in_workspace']}
        
        Given the command: "{command}"
        
        Respond in JSON format with the following structure:
        {{
            "interpretation": "Brief explanation of how you interpreted the command",
            "action_plan": [
                {{"action": "action_name", "parameters": {{"param1": "value1", ...}}}}
            ],
            "confidence": float between 0 and 1
        }}
        
        Only use actions from the provided list. If the command cannot be translated to available actions, 
        indicate so in the interpretation and return an empty action plan.
        """
        
        try:
            # Call the LLM (using OpenAI API as an example)
            response = openai.ChatCompletion.create(
                model=self.llm_model_name,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1  # Low temperature for more consistent outputs
            )
            
            # Parse the response
            response_text = response.choices[0].message.content.strip()
            
            # Extract JSON from response (in case the LLM adds text around it)
            json_start = response_text.find('{')
            json_end = response_text.rfind('}') + 1
            json_str = response_text[json_start:json_end]
            
            return json.loads(json_str)
        except Exception as e:
            rospy.logerr(f"Error interpreting command: {e}")
            return {
                "interpretation": f"Error processing command: {str(e)}",
                "action_plan": [],
                "confidence": 0.0
            }

    def execute_action_plan(self, action_plan: List[Dict], monitor_execution: bool = True) -> bool:
        """
        Execute a planned sequence of actions.
        
        Args:
            action_plan: List of actions to execute
            monitor_execution: Whether to monitor execution and handle failures
            
        Returns:
            True if the plan executed successfully, False otherwise
        """
        for i, step in enumerate(action_plan):
            action = step["action"]
            params = step.get("parameters", {})
            
            rospy.loginfo(f"Executing action {i+1}/{len(action_plan)}: {action} with {params}")
            
            success = self.execute_single_action(action, params)
            
            if not success:
                rospy.logerr(f"Action failed: {action} with {params}")
                
                if monitor_execution:
                    # Try to recover or ask for human assistance
                    return self.handle_action_failure(i, action_plan)
                else:
                    return False
                    
        return True
    
    def execute_single_action(self, action: str, params: Dict) -> bool:
        """
        Execute a single robot action.
        
        Args:
            action: Name of the action to execute
            params: Parameters for the action
            
        Returns:
            True if the action was successful, False otherwise
        """
        try:
            if action == "move_to_location":
                return self.move_to_location(params.get("x", 0.0), params.get("y", 0.0))
            elif action == "rotate_to_angle":
                return self.rotate_to_angle(params.get("theta", 0.0))
            elif action == "open_gripper":
                return self.open_gripper()
            elif action == "close_gripper":
                return self.close_gripper()
            elif action == "pick_up_object":
                return self.pick_up_object(params.get("object_name", ""))
            elif action == "place_object":
                return self.place_object(params.get("object_name", ""), params.get("location", {}))
            elif action == "navigate_to":
                return self.navigate_to(params.get("location_name", ""))
            elif action == "look_at":
                return self.look_at(params.get("location", {}))
            elif action == "detect_objects":
                return self.detect_objects()
            else:
                rospy.logerr(f"Unknown action: {action}")
                return False
        except Exception as e:
            rospy.logerr(f"Error executing action {action}: {e}")
            return False
    
    def move_to_location(self, x: float, y: float) -> bool:
        """
        Move the robot to a specific (x, y) location.
        """
        # In a real implementation, this would call navigation stack
        rospy.loginfo(f"Moving to location ({x}, {y})")
        
        # Update internal state
        self.current_state["location"]["x"] = x
        self.current_state["location"]["y"] = y
        
        # Simulate successful execution
        return True
    
    def rotate_to_angle(self, theta: float) -> bool:
        """
        Rotate the robot to a specific angle (in radians).
        """
        rospy.loginfo(f"Rotating to angle {theta} radians")
        
        # Update internal state
        self.current_state["location"]["theta"] = theta
        
        # Simulate successful execution
        return True
    
    def open_gripper(self) -> bool:
        """
        Open the robot gripper.
        """
        rospy.loginfo("Opening gripper")
        
        # Update internal state
        self.current_state["gripper"] = "open"
        
        # Simulate successful execution
        return True
    
    def close_gripper(self) -> bool:
        """
        Close the robot gripper.
        """
        rospy.loginfo("Closing gripper")
        
        # Update internal state
        self.current_state["gripper"] = "closed"
        
        # Simulate successful execution
        return True
    
    def pick_up_object(self, object_name: str) -> bool:
        """
        Pick up an object by name.
        """
        rospy.loginfo(f"Picking up object: {object_name}")
        
        # In a real implementation, this would involve perception and manipulation
        # For now, we'll just update state
        
        # Remove object from workspace and add to gripper
        if object_name in self.current_state["objects_in_workspace"]:
            self.current_state["objects_in_workspace"].remove(object_name)
        
        # Simulate successful execution
        return True
    
    def place_object(self, object_name: str, location: Dict) -> bool:
        """
        Place an object at a specific location.
        """
        rospy.loginfo(f"Placing object: {object_name} at location {location}")
        
        # In a real implementation, this would involve navigation and manipulation
        
        # Simulate successful execution
        return True
    
    def navigate_to(self, location_name: str) -> bool:
        """
        Navigate to a named location.
        """
        rospy.loginfo(f"Navigating to: {location_name}")
        
        # In a real implementation, this would use the navigation stack
        # For now, we'll just simulate navigation
        
        # Simulate successful execution
        return True
    
    def look_at(self, location: Dict) -> bool:
        """
        Look at a specific location.
        """
        rospy.loginfo(f"Looking at: {location}")
        
        # In a real implementation, this would control camera/pan-tilt units
        # For now, we'll just simulate looking
        
        # Simulate successful execution
        return True
    
    def detect_objects(self) -> bool:
        """
        Detect objects in the current view.
        """
        rospy.loginfo("Detecting objects in current view")
        
        # In a real implementation, this would use perception systems
        # For now, we'll return dummy objects
        
        # Simulate detection of some objects
        self.current_state["objects_in_workspace"] = ["red_cube", "blue_cylinder", "green_sphere"]
        
        # Simulate successful execution
        return True
    
    def handle_action_failure(self, failed_step: int, action_plan: List[Dict]) -> bool:
        """
        Attempt to handle failure in action execution.
        
        Args:
            failed_step: Index of the failed action
            action_plan: Original action plan that is being executed
            
        Returns:
            True if recovery was successful, False otherwise
        """
        rospy.logwarn(f"Action failed at step {failed_step}: {action_plan[failed_step]}")
        
        # For now, implement basic retry logic
        # In practice, you might implement more sophisticated recovery strategies
        failed_action = action_plan[failed_step]
        
        # Retry the failed action once
        rospy.loginfo(f"Retrying action: {failed_action}")
        success = self.execute_single_action(failed_action["action"], failed_action.get("parameters", {}))
        
        if success:
            rospy.loginfo("Action succeeded on retry")
            # Continue with the rest of the plan
            remaining_plan = action_plan[failed_step + 1:]
            return self.execute_action_plan(remaining_plan, monitor_execution=True)
        else:
            rospy.logerr(f"Action failed even after retry: {failed_action}")
            return False


class LLMRobotInterface:
    """
    A ROS node that interfaces with an LLM-based action planner.
    """
    def __init__(self):
        rospy.init_node('llm_robot_interface', anonymous=True)
        
        # Initialize the action planner
        self.planner = LanguageActionPlanner(
            robot_description="Differential drive robot with manipulator arm and camera"
        )
        
        # Subscribers
        self.command_sub = rospy.Subscriber('/natural_language_command', String, self.command_callback)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        # Publishers
        self.status_pub = rospy.Publisher('/llm_action_status', String, queue_size=10)
        
        # Internal state
        self.latest_image = None
        
        rospy.loginfo("LLM-Robot interface initialized")
    
    def command_callback(self, msg):
        """
        Handle incoming natural language commands.
        """
        command = msg.data
        rospy.loginfo(f"Received command: {command}")
        
        # Interpret the command to generate action plan
        plan_result = self.planner.interpret_command(command)
        
        if plan_result["confidence"] > 0.5:  # Accept plans with confidence > 0.5
            rospy.loginfo(f"Generated plan: {plan_result['action_plan']}")
            
            # Execute the action plan
            success = self.planner.execute_action_plan(plan_result["action_plan"])
            
            if success:
                rospy.loginfo("Action plan executed successfully")
                self.status_pub.publish(String(data="success"))
            else:
                rospy.logerr("Action plan execution failed")
                self.status_pub.publish(String(data="failure"))
        else:
            rospy.logwarn(f"Low confidence plan rejected: {plan_result['confidence']}")
            self.status_pub.publish(String(data="low_confidence"))
    
    def image_callback(self, msg):
        """
        Handle incoming camera images for perception.
        """
        self.latest_image = msg
        # Perception processing would happen here if needed for grounding
    
    def run(self):
        """
        Run the LLM-robot interface node.
        """
        rospy.loginfo("LLM-Robot interface node running")
        rospy.spin()


def main():
    """
    Main function to run the LLM-robot interface.
    """
    interface = LLMRobotInterface()
    interface.run()


if __name__ == '__main__':
    main()
```

## Advanced Language-to-Action Techniques

### Chain-of-Thought Reasoning

LLMs can be prompted to think through complex tasks step-by-step before generating actions:

```python
def generate_chain_of_thought_plan(self, command: str) -> Dict:
    """
    Generate a plan using chain-of-thought reasoning for complex tasks.
    """
    prompt = f"""
    You are an intelligent robot assistant that breaks down complex commands into step-by-step plans.
    
    Command: "{command}"
    
    Think through this step-by-step:
    1. What does the user want to achieve?
    2. What objects might be involved?
    3. What locations might be relevant?
    4. What sequence of actions would accomplish this?
    
    Then provide the action plan in the required JSON format.
    """
    
    # Use this prompt with the LLM to encourage step-by-step thinking
    # Implementation would be similar to interpret_command but with more detailed reasoning
    pass
```

### Grounded Language Understanding

To connect language to perception, we can incorporate visual information:

```python
def grounded_interpret_command(self, command: str, image: Optional[Image] = None) -> Dict:
    """
    Interpret a command with grounding in visual perception.
    
    Args:
        command: Natural language command
        image: Optional image to provide visual context
        
    Returns:
        Enhanced action plan with visual grounding
    """
    # If we have an image, we can enhance the prompt with visual context
    visual_context = ""
    if image:
        # In practice, we would use vision models to extract relevant information
        visual_context = "The robot sees: [extracted visual information would go here]"
    
    prompt = f"""
    You are an intelligent robot assistant that understands commands in the context of the visible environment.
    
    {visual_context}
    
    Command: "{command}"
    
    Generate an action plan that takes into account the visual environment.
    """
    
    # Implementation would follow similar pattern to interpret_command
    # but with visual context integrated
    pass
```

## Language Model Selection for Robotics

Different LLMs have different trade-offs for robotic applications:

### Smaller Models (e.g., Mistral, Phi-2)
- Faster inference, suitable for real-time applications
- Lower computational requirements
- May have less sophisticated reasoning

### Larger Models (e.g., GPT-4, Claude)
- Better reasoning and complex command understanding
- Higher computational requirements
- Potentially better generalization

### Specialized Models
- Models fine-tuned on robotics data (e.g., RT-2, PaLM-E)
- Better robot-specific task understanding
- More consistent action generation patterns

## Integration Challenges

### Latency and Real-Time Response
LLM queries can introduce significant latency. Consider approaches like:
- Caching frequent command interpretations
- Using local inference for basic commands
- Pre-planning for predictable scenarios

### Error Handling and Safety
- Validate generated actions before execution
- Implement safety checks and limits
- Design fallback behaviors for failed interpretations

### Uncertainty Management
- Represent confidence in command interpretations
- Ask for clarification when confidence is low
- Implement graceful degradation when uncertain

## Practical Implementation Tips

1. **Start Simple**: Begin with basic command mappings and gradually add complexity
2. **Iterative Refinement**: Continuously improve prompt engineering based on performance
3. **Human-in-the-Loop**: Include human oversight for safety and learning
4. **Evaluation Metrics**: Track success rates, time to completion, and user satisfaction

## Summary

In this week, you've learned:

- How to integrate large language models with robotic systems for task planning
- Techniques for converting natural language commands to executable robot actions
- Methods for grounding language understanding in perceptual context
- Approaches to handle uncertainty and failure in language-to-action translation
- Practical considerations for deploying LLMs in robotic systems

Continue to [Week 13: Course Synthesis Project](./week13.md) to apply everything you've learned across all modules in a comprehensive capstone project.