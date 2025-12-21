# Week 12: Action Planning with Large Language Models

In this week, we'll explore how large language models (LLMs) can be integrated with robotic systems to generate executable action plans from high-level natural language commands. You'll learn to connect LLMs with perception systems and low-level controllers to create intelligent robots that can follow complex instructions.

## Learning Objectives

By the end of this week, you will be able to:

- Integrate large language models with robotic systems for task planning
- Generate executable robot actions from natural language commands
- Implement grounded language understanding for robotic tasks
- Create language-conditioned action policies
- Evaluate the success of language-to-action translation in robotics

## Introduction to LLM-Robot Integration

Large Language Models (LLMs) bring powerful reasoning capabilities that can enhance robotic systems. However, bridging the gap between language understanding and physical action requires:

- **Perception Grounding**: Connecting language concepts to observed reality
- **Action Space Mapping**: Converting language to executable robot actions
- **World State Reasoning**: Maintaining awareness of robot and environment state
- **Feedback Integration**: Incorporating sensory feedback during plan execution

### LLM Integration Approaches

Two main approaches exist for connecting LLMs to robots:

1. **Direct Integration**: LLM directly outputs robot commands
2. **Symbolic Interface**: LLM generates high-level plans for a symbolic executor

## Direct Integration with LLMs

Direct integration involves prompting an LLM to generate executable robot commands:

```python
import openai
import rospy
from geometry_msgs.msg import Twist

class LLMDirectController:
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.openai_client = openai.OpenAI(api_key='YOUR_API_KEY')
        
    def execute_language_command(self, command_text):
        """Execute a natural language command directly via LLM."""
        
        # Define the robot's action space
        system_prompt = """
        You are a robot commander. You control a TurtleBot3 robot that can move forward, turn left, turn right, or stop.
        Available commands:
        - MOVE_FORWARD distance (in meters)
        - TURN_LEFT degrees
        - TURN_RIGHT degrees
        - STOP
        - GRAB_OBJECT object_name
        - PLACE_AT_LOCATION location_name
        
        Respond with ONLY the appropriate command. Do not include explanations.
        """
        
        response = self.openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": command_text}
            ],
            temperature=0.1  # Low temperature for consistent outputs
        )
        
        command = response.choices[0].message.content.strip()
        self.execute_parsed_command(command)
    
    def execute_parsed_command(self, command):
        """Execute parsed command from LLM."""
        cmd_parts = command.split()
        action = cmd_parts[0]
        
        if action == "MOVE_FORWARD":
            distance = float(cmd_parts[1])
            self.move_forward(distance)
        elif action == "TURN_LEFT":
            degrees = float(cmd_parts[1])
            self.turn_left(degrees)
        elif action == "TURN_RIGHT":
            degrees = float(cmd_parts[1])
            self.turn_right(degrees)
        elif action == "STOP":
            self.stop_robot()
        elif action == "GRAB_OBJECT":
            obj_name = cmd_parts[1]
            self.grab_object(obj_name)
        elif action == "PLACE_AT_LOCATION":
            location = " ".join(cmd_parts[1:])
            self.place_at_location(location)
    
    def move_forward(self, distance):
        """Move robot forward by specified distance."""
        # Implementation of movement with odometry feedback
        vel_msg = Twist()
        vel_msg.linear.x = 0.2  # Constant speed
        # Calculate time based on distance and speed
        duration = distance / vel_msg.linear.x
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.pub_cmd_vel.publish(vel_msg)
            rospy.sleep(0.1)
        
        self.stop_robot()
    
    def turn_left(self, degrees):
        """Turn robot left by specified degrees."""
        # Implementation of turning
        pass  # Implementation details...
    
    def stop_robot(self):
        """Stop robot movement."""
        vel_msg = Twist()
        self.pub_cmd_vel.publish(vel_msg)
```

### Challenges with Direct Integration

Direct integration faces several challenges:

- **Precision**: Language models might generate imprecise values
- **Safety**: No guaranteed safety constraints on generated commands
- **Robustness**: LLMs may generate invalid commands
- **Consistency**: Same command may generate different outputs

## Symbolic Execution Approach

A safer approach is to use LLMs to generate symbolic plans that are then executed by a symbolic executor:

```python
from dataclasses import dataclass
from enum import Enum
from typing import List, Dict

class ActionType(Enum):
    NAVIGATE_TO = "navigate_to"
    GRASP_OBJECT = "grasp_object"
    PLACE_OBJECT = "place_object"
    DETECT_OBJECT = "detect_object"
    WAIT = "wait"
    SAY = "say"

@dataclass
class Action:
    type: ActionType
    parameters: Dict[str, str]

class SymbolicPlanner:
    def __init__(self):
        self.objects_locations = {}  # Knowledge base
        self.robot_state = {}  # Current robot state

    def generate_plan(self, natural_language_goal):
        """Generate a symbolic plan from natural language."""
        
        prompt = f"""
        Given the goal: "{natural_language_goal}"
        
        Generate a sequence of symbolic actions for a robot. Each action must be from the set:
        - NAVIGATE_TO(location)
        - GRASP_OBJECT(object)
        - PLACE_OBJECT(object, location)
        - DETECT_OBJECT(object)
        - WAIT(duration)
        - SAY(text)
        
        Output format: 
        ACTION_NAME(PARAM1=value1, PARAM2=value2)
        ACTION_NAME(PARAM1=value1)
        ...
        
        Example:
        DETECT_OBJECT(name="red_cup")
        NAVIGATE_TO(location="kitchen_counter")
        GRASP_OBJECT(object="red_cup")
        NAVIGATE_TO(location="table")
        PLACE_OBJECT(object="red_cup", location="table")
        """
        
        # Call LLM to generate plan
        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.2
        )
        
        plan_text = response.choices[0].message.content
        return self.parse_plan(plan_text)
    
    def parse_plan(self, plan_text):
        """Parse plan text into structured actions."""
        actions = []
        for line in plan_text.strip().split('\n'):
            if '(' in line and ')' in line:
                action_part = line.split('(')[0].strip()
                params_part = line[line.find('(')+1:line.rfind(')')]
                
                # Convert action string to ActionType
                try:
                    action_type = ActionType[action_part.upper()]
                except KeyError:
                    raise ValueError(f"Invalid action type: {action_part}")
                
                # Parse parameters
                params = {}
                if params_part:
                    for param in params_part.split(','):
                        if '=' in param:
                            key, value = param.split('=', 1)
                            params[key.strip()] = value.strip().strip('"\'')
                
                actions.append(Action(type=action_type, parameters=params))
        
        return actions
    
    def execute_plan(self, plan: List[Action]):
        """Execute a symbolic plan safely."""
        for action in plan:
            if not self.validate_action(action):
                raise ValueError(f"Invalid action: {action}")
            
            self.execute_single_action(action)
    
    def validate_action(self, action: Action):
        """Validate action against safety constraints."""
        # Implement safety checks
        if action.type == ActionType.NAVIGATE_TO:
            location = action.parameters.get('location')
            if location not in self.get_valid_locations():
                return False
        return True
    
    def execute_single_action(self, action: Action):
        """Execute a single symbolic action."""
        if action.type == ActionType.NAVIGATE_TO:
            self.navigate_to(action.parameters['location'])
        elif action.type == ActionType.GRASP_OBJECT:
            self.grasp_object(action.parameters['object'])
        elif action.type == ActionType.PLACE_OBJECT:
            self.place_object(
                action.parameters['object'], 
                action.parameters['location']
            )
        # Add other action implementations