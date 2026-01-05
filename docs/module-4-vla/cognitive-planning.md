---
sidebar_position: 22
title: "Cognitive Planning using LLMs to convert natural language commands"
---

# Cognitive Planning using LLMs to convert natural language commands

## Overview

Cognitive planning is the process of transforming high-level natural language commands into executable action sequences for robots. Large Language Models (LLMs) serve as the cognitive bridge between human commands and robot actions, enabling sophisticated reasoning and task decomposition that accounts for environmental context and robot capabilities.

## The Cognitive Planning Process

### High-Level Architecture
```
Natural Language Command → LLM Processing → Task Decomposition → Action Sequencing → Robot Execution
```

### Cognitive Planning Pipeline
1. **Command Interpretation**: Understanding the user's intent
2. **Environmental Analysis**: Assessing the current state
3. **Task Decomposition**: Breaking complex tasks into subtasks
4. **Action Planning**: Creating sequences of executable actions
5. **Constraint Checking**: Ensuring feasibility and safety
6. **Execution Strategy**: Determining the best execution approach

## Natural Language Understanding with LLMs

### Command Interpretation Framework
```python
import openai
from typing import Dict, List, Any

class NaturalLanguageInterpreter:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.system_prompt = """
        You are a robot command interpreter. Your role is to understand human commands
        and break them down into executable robot actions. Consider the robot's
        capabilities, environmental constraints, and safety requirements.
        """

    def interpret_command(self, command: str, robot_capabilities: Dict) -> Dict:
        """
        Interpret a natural language command and return structured task breakdown
        """
        prompt = f"""
        Command: "{command}"

        Robot capabilities: {robot_capabilities}

        Please break down this command into:
        1. Intent: What the user wants to achieve
        2. Objects: What objects are involved
        3. Actions: What actions are needed
        4. Constraints: Any safety or feasibility constraints
        5. Success criteria: How to know the task is complete

        Respond in JSON format.
        """

        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": prompt}
            ],
            temperature=0.1
        )

        return self.parse_json_response(response.choices[0].message.content)
```

### Context-Aware Interpretation
```python
class ContextAwareInterpreter(NaturalLanguageInterpreter):
    def __init__(self, api_key: str):
        super().__init__(api_key)
        self.context_history = []

    def interpret_with_context(self, command: str, environment_state: Dict,
                              robot_state: Dict, conversation_history: List = None) -> Dict:
        """
        Interpret command with environmental and contextual awareness
        """
        context_prompt = f"""
        Current environment state: {environment_state}
        Current robot state: {robot_state}
        Conversation history: {conversation_history or []}

        Command: "{command}"

        Based on the current context, interpret this command and provide:
        1. Updated intent considering context
        2. Relevant objects in the environment
        3. Feasible action sequence
        4. Environmental constraints
        5. Expected outcomes
        """

        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": context_prompt}
            ]
        )

        return self.parse_json_response(response.choices[0].message.content)
```

## Task Decomposition Strategies

### Hierarchical Task Networks (HTN)
```python
class TaskDecomposer:
    def __init__(self):
        self.decomposition_rules = {
            "clean_room": [
                "identify_dirty_objects",
                "plan_navigation_to_objects",
                "execute_cleaning_actions"
            ],
            "fetch_object": [
                "locate_object",
                "plan_navigation_to_object",
                "grasp_object",
                "navigate_to_destination",
                "place_object"
            ]
        }

    def decompose_task(self, high_level_task: str, command_details: Dict) -> List[Dict]:
        """
        Decompose high-level tasks into primitive actions
        """
        if high_level_task in self.decomposition_rules:
            subtasks = []
            for subtask_name in self.decomposition_rules[high_level_task]:
                subtask = self.create_subtask(subtask_name, command_details)
                subtasks.append(subtask)
            return subtasks
        else:
            # Use LLM for novel task decomposition
            return self.llm_decompose_task(high_level_task, command_details)

    def create_subtask(self, subtask_name: str, command_details: Dict) -> Dict:
        """
        Create a structured subtask with parameters
        """
        subtask_templates = {
            "locate_object": {
                "action": "perception",
                "parameters": {
                    "object_type": command_details.get("target_object", "unknown")
                },
                "success_criteria": "object_detected"
            },
            "plan_navigation_to_object": {
                "action": "navigation",
                "parameters": {
                    "target_location": command_details.get("object_location", "unknown")
                },
                "success_criteria": "navigation_complete"
            },
            "grasp_object": {
                "action": "manipulation",
                "parameters": {
                    "object_id": command_details.get("object_id", "unknown"),
                    "grasp_type": "precision"
                },
                "success_criteria": "object_grasped"
            }
        }

        return subtask_templates.get(subtask_name, {
            "action": subtask_name,
            "parameters": {},
            "success_criteria": "completed"
        })
```

### Spatial and Temporal Reasoning
```python
class SpatialTemporalPlanner:
    def __init__(self):
        self.spatial_reasoner = SpatialReasoner()
        self.temporal_reasoner = TemporalReasoner()

    def plan_with_spatial_reasoning(self, command: str, environment_map: Dict) -> List[Dict]:
        """
        Plan actions considering spatial relationships and constraints
        """
        spatial_prompt = f"""
        Environment map: {environment_map}
        Command: {command}

        Consider spatial relationships such as:
        - Proximity requirements
        - Obstacle avoidance
        - Navigation paths
        - Manipulation reachability
        - Object affordances

        Generate a spatially-aware action plan.
        """

        # Use LLM to generate spatially-aware plan
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "user", "content": spatial_prompt}
            ]
        )

        return self.parse_action_plan(response.choices[0].message.content)

    def plan_with_temporal_reasoning(self, action_sequence: List[Dict]) -> List[Dict]:
        """
        Plan actions considering temporal dependencies and constraints
        """
        temporal_constraints = []
        for i, action in enumerate(action_sequence):
            if i > 0:
                # Check if current action depends on previous action completion
                if self.has_temporal_dependency(action_sequence[i-1], action):
                    temporal_constraints.append({
                        "before": i-1,
                        "after": i,
                        "constraint": "must_complete_before"
                    })

        return self.reorder_with_temporal_constraints(action_sequence, temporal_constraints)
```

## Action Planning and Sequencing

### ROS 2 Action Integration
```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from std_msgs.msg import String

class ActionSequencer(Node):
    def __init__(self):
        super().__init__('action_sequencer')

        # Action clients for different robot capabilities
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.moveit_client = ActionClient(self, MoveGroup, 'move_group')
        self.gripper_client = ActionClient(self, GripperCommand, 'gripper_command')

    def execute_action_sequence(self, action_sequence: List[Dict]) -> bool:
        """
        Execute a sequence of actions with monitoring and error handling
        """
        for i, action in enumerate(action_sequence):
            self.get_logger().info(f'Executing action {i+1}/{len(action_sequence)}: {action["action"]}')

            success = self.execute_single_action(action)
            if not success:
                self.get_logger().error(f'Action failed: {action["action"]}')

                # Try recovery behavior
                recovery_success = self.attempt_recovery(action, action_sequence, i)
                if not recovery_success:
                    return False  # Task failed completely

        return True  # All actions completed successfully

    def execute_single_action(self, action: Dict) -> bool:
        """
        Execute a single action based on its type
        """
        action_type = action['action']

        if action_type == 'navigation':
            return self.execute_navigation_action(action['parameters'])
        elif action_type == 'manipulation':
            return self.execute_manipulation_action(action['parameters'])
        elif action_type == 'perception':
            return self.execute_perception_action(action['parameters'])
        else:
            self.get_logger().error(f'Unknown action type: {action_type}')
            return False

    def execute_navigation_action(self, params: Dict) -> bool:
        """
        Execute navigation action
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = Pose()  # Set from parameters
        goal_msg.pose.position.x = params.get('x', 0.0)
        goal_msg.pose.position.y = params.get('y', 0.0)
        goal_msg.pose.position.z = params.get('z', 0.0)

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)

        # Wait for result with timeout
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        return future.result().result.success
```

## Constraint Checking and Safety Validation

### Safety Constraint Validator
```python
class SafetyConstraintValidator:
    def __init__(self):
        self.safety_rules = {
            "human_proximity": 0.5,  # Minimum distance from humans
            "speed_limits": {"navigation": 0.5, "manipulation": 0.1},  # m/s
            "force_limits": {"gripper": 50.0},  # Newtons
            "workspace_bounds": {"x": (-2.0, 2.0), "y": (-2.0, 2.0), "z": (0.0, 1.5)}
        }

    def validate_action_sequence(self, action_sequence: List[Dict],
                                environment_state: Dict) -> Tuple[bool, List[str]]:
        """
        Validate action sequence against safety constraints
        """
        violations = []

        for action in action_sequence:
            action_violations = self.check_action_safety(action, environment_state)
            violations.extend(action_violations)

        return len(violations) == 0, violations

    def check_action_safety(self, action: Dict, environment_state: Dict) -> List[str]:
        """
        Check if a single action violates safety constraints
        """
        violations = []

        # Check human proximity
        if action['action'] == 'navigation':
            target_pose = action.get('parameters', {}).get('pose', {})
            humans_nearby = environment_state.get('humans', [])

            for human in humans_nearby:
                distance = self.calculate_distance(target_pose, human['pose'])
                if distance < self.safety_rules['human_proximity']:
                    violations.append(f"Navigation target too close to human: {distance:.2f}m")

        # Check workspace bounds
        if 'pose' in action.get('parameters', {}):
            pose = action['parameters']['pose']
            bounds = self.safety_rules['workspace_bounds']

            if not (bounds['x'][0] <= pose.get('x', 0) <= bounds['x'][1]):
                violations.append(f"X position {pose.get('x', 0)} outside bounds {bounds['x']}")

        return violations
```

## Adaptive Planning and Error Recovery

### Plan Adaptation Framework
```python
class AdaptivePlanner:
    def __init__(self):
        self.execution_monitor = ExecutionMonitor()
        self.recovery_strategies = {
            "navigation_failure": self.recovery_navigation_failure,
            "grasp_failure": self.recovery_grasp_failure,
            "object_not_found": self.recovery_object_not_found
        }

    def execute_with_adaptation(self, action_sequence: List[Dict]) -> bool:
        """
        Execute action sequence with real-time adaptation capabilities
        """
        current_index = 0
        max_attempts = 3

        while current_index < len(action_sequence):
            action = action_sequence[current_index]
            attempts = 0

            while attempts < max_attempts:
                success = self.execute_single_action_with_monitoring(action)

                if success:
                    current_index += 1
                    break
                else:
                    attempts += 1
                    self.get_logger().info(f'Attempt {attempts} failed for action: {action["action"]}')

                    if attempts >= max_attempts:
                        # Try recovery strategy
                        recovery_success = self.attempt_recovery(action, action_sequence, current_index)
                        if not recovery_success:
                            return False
                        else:
                            # Recovery successful, continue with next action
                            current_index += 1
                            break
                    else:
                        # Brief pause before retry
                        time.sleep(1.0)

        return True

    def attempt_recovery(self, failed_action: Dict, full_sequence: List[Dict],
                        current_index: int) -> bool:
        """
        Attempt to recover from action failure
        """
        failure_type = self.classify_failure(failed_action)

        if failure_type in self.recovery_strategies:
            return self.recovery_strategies[failure_type](failed_action, full_sequence, current_index)
        else:
            self.get_logger().error(f'No recovery strategy for failure type: {failure_type}')
            return False
```

## Integration with Perception Systems

### Perception-Action Coordination
```python
class PerceptionActionCoordinator:
    def __init__(self):
        self.perception_client = PerceptionClient()
        self.action_sequencer = ActionSequencer()

    def execute_perception_aware_plan(self, command: str, environment_state: Dict) -> bool:
        """
        Execute a plan that adapts based on real-time perception
        """
        # Get initial plan from LLM
        plan = self.generate_plan_with_llm(command, environment_state)

        for action in plan:
            if action['action'] == 'perception':
                # Execute perception action
                perception_result = self.execute_perception_action(action['parameters'])

                # Update environment state based on perception
                environment_state = self.update_environment_state(
                    environment_state, perception_result)

                # Potentially replan based on new information
                if self.should_replan(action, perception_result):
                    updated_plan = self.generate_plan_with_llm(
                        command, environment_state, previous_plan=plan)
                    plan = self.merge_plans(plan, updated_plan, action)

            else:
                # Execute non-perception action
                success = self.action_sequencer.execute_single_action(action)
                if not success:
                    return False

        return True

    def should_replan(self, action: Dict, perception_result: Dict) -> bool:
        """
        Determine if replanning is needed based on perception result
        """
        # Replan if object not found when expected
        if (action['action'] == 'locate_object' and
            not perception_result.get('object_found', False)):
            return True

        # Replan if environment changed significantly
        if perception_result.get('significant_change', False):
            return True

        return False
```

## Performance Optimization

### Caching and Planning Efficiency
```python
from functools import lru_cache
import hashlib

class EfficientCognitivePlanner:
    def __init__(self):
        self.plan_cache = {}
        self.max_cache_size = 1000

    @lru_cache(maxsize=128)
    def get_cached_plan(self, command_hash: str) -> Dict:
        """
        Get previously computed plan for identical command
        """
        pass

    def plan_with_caching(self, command: str, context: Dict) -> Dict:
        """
        Generate plan with caching to improve performance
        """
        # Create hash of command and relevant context
        cache_key = self.create_cache_key(command, context)

        # Check if plan exists in cache
        if cache_key in self.plan_cache:
            return self.plan_cache[cache_key]

        # Generate new plan
        plan = self.generate_fresh_plan(command, context)

        # Cache the plan
        if len(self.plan_cache) < self.max_cache_size:
            self.plan_cache[cache_key] = plan

        return plan

    def create_cache_key(self, command: str, context: Dict) -> str:
        """
        Create a hash key for caching plans
        """
        context_str = str(sorted(context.items()))
        combined = f"{command}_{context_str}"
        return hashlib.md5(combined.encode()).hexdigest()
```

## Best Practices for Cognitive Planning

### Design Principles
- **Modularity**: Separate command interpretation, task decomposition, and action execution
- **Robustness**: Handle ambiguous commands and unexpected situations
- **Safety**: Always validate actions against safety constraints
- **Adaptability**: Adjust plans based on real-world feedback
- **Efficiency**: Optimize for real-time performance

### Implementation Guidelines
- **Error Handling**: Implement comprehensive error detection and recovery
- **State Management**: Maintain accurate robot and environment state
- **Validation**: Verify plan feasibility before execution
- **Monitoring**: Continuously monitor execution progress
- **Logging**: Maintain detailed execution logs for debugging

## Summary

Cognitive planning using LLMs enables robots to understand and execute complex natural language commands by breaking them down into executable action sequences. This process involves command interpretation, task decomposition, action sequencing, and constraint checking. By integrating with perception systems and implementing adaptive planning capabilities, robots can successfully execute tasks in dynamic environments while maintaining safety and reliability.