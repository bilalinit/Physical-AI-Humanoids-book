---
sidebar_position: 10
title: 'Chapter 10: The Brain (LLM Action Planning)'
description: 'Implementing Large Language Model-based action planning for humanoid robots'
---

# Chapter 10: The Brain (LLM Action Planning)

## Introduction

The integration of Large Language Models (LLMs) as the cognitive engine for humanoid robots represents a paradigm shift in robotics. This chapter explores how to leverage LLMs like GPT-4 for high-level action planning, enabling humanoid robots to interpret natural language commands, reason about complex tasks, and generate executable action sequences. We'll cover the architecture, implementation patterns, and practical considerations for deploying LLM-based planning in humanoid robotics.

## LLM Integration Architecture

### Cognitive Architecture Overview

The LLM-based cognitive system for humanoid robots consists of several interconnected components:

```python
import openai
import json
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
from enum import Enum

class ActionStatus(Enum):
    PENDING = "pending"
    EXECUTING = "executing"
    SUCCESS = "success"
    FAILED = "failed"
    CANCELLED = "cancelled"

@dataclass
class Action:
    """Represents an atomic action for the robot"""
    id: str
    name: str
    parameters: Dict[str, Any]
    description: str
    prerequisites: List[str]
    effects: List[str]

@dataclass
class PlanStep:
    """A step in the execution plan"""
    action: Action
    status: ActionStatus
    result: Optional[Dict[str, Any]] = None
    error: Optional[str] = None

class LLMCognitiveEngine:
    """Main cognitive engine using LLM for planning"""
    def __init__(self, api_key: str, model: str = "gpt-4-turbo"):
        openai.api_key = api_key
        self.model = model
        self.action_registry = ActionRegistry()
        self.context_memory = ContextMemory()

    def plan_action_sequence(self, goal: str, context: Dict[str, Any]) -> List[PlanStep]:
        """Generate a sequence of actions to achieve the goal"""
        # Create a prompt for the LLM
        prompt = self._create_planning_prompt(goal, context)

        # Get response from LLM
        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self._get_system_prompt()},
                {"role": "user", "content": prompt}
            ],
            functions=self._get_available_functions(),
            function_call="auto"
        )

        # Parse the response and create action sequence
        return self._parse_llm_response(response, goal)

    def _create_planning_prompt(self, goal: str, context: Dict[str, Any]) -> str:
        """Create a prompt for the LLM to generate an action plan"""
        return f"""
        You are a planning assistant for a humanoid robot. Your task is to create a detailed action plan to achieve the following goal:

        GOAL: {goal}

        CURRENT CONTEXT:
        - Robot capabilities: {context.get('capabilities', 'unknown')}
        - Environment: {context.get('environment', 'unknown')}
        - Available objects: {context.get('objects', 'none')}
        - Robot state: {context.get('state', 'unknown')}
        - Previous actions: {context.get('previous_actions', 'none')}

        Please generate a step-by-step action plan that:
        1. Breaks down the complex goal into simple, executable actions
        2. Considers the robot's capabilities and current context
        3. Ensures each action is feasible and safe
        4. Accounts for potential obstacles or constraints

        Return the plan as a sequence of actions with parameters.
        """
```

### Action Registry and Capability Management

```python
class ActionRegistry:
    """Manages available actions and their capabilities"""
    def __init__(self):
        self.actions = {}
        self.capability_mapping = {}
        self._initialize_default_actions()

    def _initialize_default_actions(self):
        """Initialize default robot actions"""
        default_actions = [
            Action(
                id="move_to",
                name="move_to",
                parameters={"location": "str", "speed": "float"},
                description="Move robot to specified location",
                prerequisites=[],
                effects=["robot_at_location"]
            ),
            Action(
                id="grasp_object",
                name="grasp_object",
                parameters={"object_id": "str", "arm": "str"},
                description="Grasp an object with specified arm",
                prerequisites=["robot_at_object_location"],
                effects=["object_grasped"]
            ),
            Action(
                id="place_object",
                name="place_object",
                parameters={"location": "str", "arm": "str"},
                description="Place object at specified location",
                prerequisites=["object_grasped"],
                effects=["object_placed"]
            ),
            Action(
                id="navigate_room",
                name="navigate_room",
                parameters={"room": "str", "waypoints": "list"},
                description="Navigate through a room using waypoints",
                prerequisites=[],
                effects=["robot_navigated"]
            ),
            Action(
                id="detect_object",
                name="detect_object",
                parameters={"object_type": "str", "search_area": "str"},
                description="Detect specific object in search area",
                prerequisites=[],
                effects=["object_detected"]
            ),
            Action(
                id="speak",
                name="speak",
                parameters={"text": "str", "language": "str"},
                description="Speak text using TTS",
                prerequisites=[],
                effects=["spoken"]
            )
        ]

        for action in default_actions:
            self.actions[action.id] = action
            self.capability_mapping[action.name] = action.id

    def get_available_actions(self) -> List[Action]:
        """Get all available actions"""
        return list(self.actions.values())

    def get_action_by_name(self, name: str) -> Optional[Action]:
        """Get action by name"""
        action_id = self.capability_mapping.get(name)
        return self.actions.get(action_id) if action_id else None

    def register_action(self, action: Action):
        """Register a new action"""
        self.actions[action.id] = action
        self.capability_mapping[action.name] = action.id
```

## Context Management and Memory

### Context Memory System

```python
from datetime import datetime, timedelta
import pickle

class ContextMemory:
    """Manages robot context and memory"""
    def __init__(self, max_memory_items: int = 100):
        self.max_memory_items = max_memory_items
        self.memory_items = []
        self.object_locations = {}
        self.robot_state = {}
        self.task_history = []

    def add_memory(self, key: str, value: Any, timestamp: datetime = None):
        """Add an item to memory"""
        if timestamp is None:
            timestamp = datetime.now()

        memory_item = {
            'key': key,
            'value': value,
            'timestamp': timestamp,
            'expiry': timestamp + timedelta(hours=24)  # 24 hour expiry
        }

        self.memory_items.append(memory_item)

        # Keep memory size manageable
        if len(self.memory_items) > self.max_memory_items:
            self.memory_items.pop(0)

        # Clean expired items
        self._clean_expired_items()

    def get_memory(self, key: str) -> Optional[Any]:
        """Retrieve an item from memory"""
        for item in reversed(self.memory_items):  # Most recent first
            if item['key'] == key and item['timestamp'] < item['expiry']:
                return item['value']
        return None

    def _clean_expired_items(self):
        """Remove expired memory items"""
        current_time = datetime.now()
        self.memory_items = [
            item for item in self.memory_items
            if item['timestamp'] < item['expiry']
        ]

    def update_robot_state(self, new_state: Dict[str, Any]):
        """Update robot state"""
        self.robot_state.update(new_state)

    def get_context_for_planning(self) -> Dict[str, Any]:
        """Get relevant context for planning"""
        return {
            'robot_state': self.robot_state,
            'object_locations': self.object_locations,
            'recent_actions': self.task_history[-5:],  # Last 5 actions
            'memory_summary': self._get_memory_summary()
        }

    def _get_memory_summary(self) -> str:
        """Get a summary of recent memory items"""
        recent_items = self.memory_items[-10:]  # Last 10 items
        summary = []
        for item in recent_items:
            summary.append(f"{item['key']}: {str(item['value'])[:100]}...")  # Truncate long values
        return "; ".join(summary)
```

## LLM-Based Planning Implementation

### Plan Generation with LLM

```python
import json
import re
from typing import Union

class PlanGenerator:
    def __init__(self, cognitive_engine: LLMCognitiveEngine):
        self.engine = cognitive_engine

    def generate_plan(self, goal: str, context: Dict[str, Any]) -> List[PlanStep]:
        """Generate a plan using LLM"""
        # Create system prompt
        system_prompt = f"""
        You are an advanced action planning system for a humanoid robot. Your role is to:
        1. Decompose high-level goals into executable action sequences
        2. Consider robot capabilities, current state, and environmental constraints
        3. Generate safe, feasible action plans
        4. Account for potential failures and contingencies

        Available actions: {self._get_available_actions_description()}

        Format your response as a JSON array of actions with the following structure:
        {{
            "plan": [
                {{
                    "action": "action_name",
                    "parameters": {{"param1": "value1", "param2": "value2"}},
                    "description": "Brief description of the action"
                }}
            ]
        }}
        """

        # Create user prompt
        user_prompt = f"""
        Goal: {goal}

        Context:
        - Robot capabilities: {context.get('capabilities', 'unknown')}
        - Current environment: {context.get('environment', 'unknown')}
        - Available objects: {context.get('objects', 'none')}
        - Robot state: {context.get('state', 'unknown')}
        - Constraints: {context.get('constraints', 'none')}

        Generate a detailed action plan to achieve this goal.
        """

        try:
            response = openai.ChatCompletion.create(
                model=self.engine.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.1,  # Low temperature for more deterministic output
                max_tokens=1000
            )

            # Extract and parse the response
            response_text = response.choices[0].message.content.strip()

            # Try to extract JSON from the response
            json_match = re.search(r'\{.*\}', response_text, re.DOTALL)
            if json_match:
                json_str = json_match.group(0)
                plan_data = json.loads(json_str)

                # Convert to PlanStep objects
                plan_steps = []
                for i, action_data in enumerate(plan_data.get('plan', [])):
                    action = self.engine.action_registry.get_action_by_name(action_data['action'])
                    if action:
                        # Create a new action with parameters from LLM
                        llm_action = Action(
                            id=f"action_{i}",
                            name=action_data['action'],
                            parameters=action_data.get('parameters', {}),
                            description=action_data.get('description', ''),
                            prerequisites=action.prerequisites,
                            effects=action.effects
                        )
                        plan_steps.append(PlanStep(action=llm_action, status=ActionStatus.PENDING))
                    else:
                        print(f"Warning: Unknown action '{action_data['action']}' in plan")

                return plan_steps
            else:
                raise ValueError(f"Could not extract JSON from LLM response: {response_text}")

        except Exception as e:
            print(f"Error generating plan: {e}")
            return self._fallback_plan(goal, context)

    def _get_available_actions_description(self) -> str:
        """Get description of available actions for LLM"""
        actions = self.engine.action_registry.get_available_actions()
        descriptions = []
        for action in actions:
            descriptions.append(
                f"- {action.name}: {action.description} "
                f"Parameters: {list(action.parameters.keys()) if action.parameters else 'none'}"
            )
        return "\n".join(descriptions)

    def _fallback_plan(self, goal: str, context: Dict[str, Any]) -> List[PlanStep]:
        """Generate a fallback plan if LLM fails"""
        # Simple fallback based on keywords in goal
        goal_lower = goal.lower()

        if 'move' in goal_lower or 'go' in goal_lower or 'navigate' in goal_lower:
            return [PlanStep(
                action=Action(
                    id="fallback_move",
                    name="move_to",
                    parameters={"location": "default", "speed": 0.5},
                    description="Fallback movement action",
                    prerequisites=[],
                    effects=["robot_moved"]
                ),
                status=ActionStatus.PENDING
            )]

        elif 'grasp' in goal_lower or 'pick' in goal_lower or 'take' in goal_lower:
            return [PlanStep(
                action=Action(
                    id="fallback_grasp",
                    name="grasp_object",
                    parameters={"object_id": "default", "arm": "right"},
                    description="Fallback grasping action",
                    prerequisites=[],
                    effects=["object_grasped"]
                ),
                status=ActionStatus.PENDING
            )]

        else:
            return [PlanStep(
                action=Action(
                    id="fallback_speak",
                    name="speak",
                    parameters={"text": f"I don't know how to {goal}", "language": "en"},
                    description="Fallback speaking action",
                    prerequisites=[],
                    effects=["spoken"]
                ),
                status=ActionStatus.PENDING
            )]
```

## Plan Execution and Monitoring

### Plan Executor

```python
import asyncio
import time
from typing import Callable, Optional

class PlanExecutor:
    def __init__(self, cognitive_engine: LLMCognitiveEngine):
        self.engine = cognitive_engine
        self.robot_interface = None  # To be set externally
        self.current_plan: List[PlanStep] = []
        self.current_step_index = 0
        self.execution_callbacks: Dict[str, Callable] = {}

    def set_robot_interface(self, robot_interface):
        """Set the interface to communicate with the actual robot"""
        self.robot_interface = robot_interface

    def execute_plan(self, plan: List[PlanStep]) -> bool:
        """Execute a plan step by step"""
        self.current_plan = plan
        self.current_step_index = 0

        success = True
        for i, step in enumerate(plan):
            print(f"Executing step {i+1}/{len(plan)}: {step.action.name}")
            step.status = ActionStatus.EXECUTING

            try:
                # Execute the action
                result = self._execute_action(step.action)
                step.result = result
                step.status = ActionStatus.SUCCESS

                # Trigger any callbacks
                self._trigger_callbacks(step.action.name, result)

                print(f"Action {step.action.name} completed successfully")

            except Exception as e:
                step.error = str(e)
                step.status = ActionStatus.FAILED
                print(f"Action {step.action.name} failed: {e}")
                success = False
                break  # Stop execution on failure

            self.current_step_index += 1

        return success

    def _execute_action(self, action: Action) -> Dict[str, Any]:
        """Execute a single action on the robot"""
        if not self.robot_interface:
            raise RuntimeError("Robot interface not set")

        # Map action name to robot interface method
        method_name = f"execute_{action.name}"
        if hasattr(self.robot_interface, method_name):
            method = getattr(self.robot_interface, method_name)
            return method(**action.parameters)
        else:
            # Try generic execution
            return self._generic_execute(action)

    def _generic_execute(self, action: Action) -> Dict[str, Any]:
        """Generic execution method for unknown actions"""
        # In a real implementation, this would use ROS services or similar
        print(f"Executing generic action: {action.name} with parameters: {action.parameters}")

        # Simulate execution
        time.sleep(0.5)  # Simulate execution time

        return {
            "status": "success",
            "execution_time": 0.5,
            "action": action.name,
            "parameters": action.parameters
        }

    def add_callback(self, action_name: str, callback: Callable):
        """Add a callback for when an action is executed"""
        self.execution_callbacks[action_name] = callback

    def _trigger_callbacks(self, action_name: str, result: Dict[str, Any]):
        """Trigger callbacks for executed action"""
        if action_name in self.execution_callbacks:
            callback = self.execution_callbacks[action_name]
            callback(result)

    def cancel_current_plan(self):
        """Cancel the current plan execution"""
        for step in self.current_plan[self.current_step_index:]:
            if step.status == ActionStatus.PENDING:
                step.status = ActionStatus.CANCELLED
```

## Reasoning and Adaptation

### Plan Adaptation System

```python
class PlanAdaptationSystem:
    def __init__(self, cognitive_engine: LLMCognitiveEngine):
        self.engine = cognitive_engine
        self.failure_history = []

    def adapt_plan_on_failure(self, failed_step: PlanStep, context: Dict[str, Any]) -> Optional[List[PlanStep]]:
        """Adapt the plan when an action fails"""
        # Log the failure
        self.failure_history.append({
            'action': failed_step.action.name,
            'error': failed_step.error,
            'context': context,
            'timestamp': datetime.now()
        })

        # Use LLM to suggest alternative approach
        adaptation_prompt = f"""
        The following action failed during plan execution:
        Action: {failed_step.action.name}
        Parameters: {failed_step.action.parameters}
        Error: {failed_step.error}

        Current context: {context}

        Please suggest an alternative approach or modification to the plan to handle this failure.
        Consider:
        1. Alternative actions that could achieve the same effect
        2. Modifications to parameters of the failed action
        3. Additional steps needed to recover from the failure
        4. Prevention of similar failures in the future

        Return your suggestion as a JSON object with the following structure:
        {{
            "alternative_actions": [
                {{
                    "action": "action_name",
                    "parameters": {{"param1": "value1"}},
                    "reason": "Why this alternative is suggested"
                }}
            ],
            "modified_plan": [...],  // Modified plan if needed
            "recovery_steps": [...]  // Steps to recover from failure
        }}
        """

        try:
            response = openai.ChatCompletion.create(
                model=self.engine.model,
                messages=[
                    {"role": "system", "content": "You are a plan adaptation system for humanoid robots. Your role is to suggest alternatives when actions fail."},
                    {"role": "user", "content": adaptation_prompt}
                ],
                temperature=0.3
            )

            response_text = response.choices[0].message.content.strip()
            json_match = re.search(r'\{.*\}', response_text, re.DOTALL)

            if json_match:
                adaptation_data = json.loads(json_match.group(0))

                # Create new plan steps based on LLM suggestion
                new_steps = []
                for alt_action in adaptation_data.get('alternative_actions', []):
                    action = self.engine.action_registry.get_action_by_name(alt_action['action'])
                    if action:
                        new_action = Action(
                            id=f"adapted_{action.id}",
                            name=alt_action['action'],
                            parameters=alt_action.get('parameters', {}),
                            description=alt_action.get('reason', ''),
                            prerequisites=action.prerequisites,
                            effects=action.effects
                        )
                        new_steps.append(PlanStep(action=new_action, status=ActionStatus.PENDING))

                return new_steps

        except Exception as e:
            print(f"Error adapting plan: {e}")
            return None

    def learn_from_failures(self) -> Dict[str, Any]:
        """Analyze failure patterns to improve future planning"""
        if not self.failure_history:
            return {}

        # Analyze patterns in failures
        failure_counts = {}
        for failure in self.failure_history:
            action = failure['action']
            failure_counts[action] = failure_counts.get(action, 0) + 1

        # Identify most common failures
        most_common_failures = sorted(failure_counts.items(), key=lambda x: x[1], reverse=True)[:5]

        # Generate learning insights
        insights = {
            'common_failures': most_common_failures,
            'total_failures': len(self.failure_history),
            'failure_rate_by_action': failure_counts
        }

        return insights
```

## Integration with ROS 2

### ROS 2 Action Client Integration

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from move_base_msgs.action import MoveBase
from std_msgs.msg import String

class ROS2RobotInterface(Node):
    def __init__(self):
        super().__init__('llm_robot_interface')

        # Action clients for different robot capabilities
        self.move_client = ActionClient(self, MoveBase, 'move_base')
        self.speech_publisher = self.create_publisher(String, 'tts_input', 10)

    def execute_move_to(self, location: str, speed: float = 0.5) -> Dict[str, Any]:
        """Execute move_to action using ROS 2"""
        # Convert location string to coordinates (this would be more sophisticated in practice)
        pose = self._location_to_pose(location)

        goal_msg = MoveBase.Goal()
        goal_msg.target_pose = pose
        goal_msg.speed = speed

        # Send goal
        self.move_client.wait_for_server()
        future = self.move_client.send_goal_async(goal_msg)

        # Wait for result
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        return {
            'status': 'success' if result else 'failed',
            'result': result
        }

    def execute_speak(self, text: str, language: str = "en") -> Dict[str, Any]:
        """Execute speak action using ROS 2"""
        msg = String()
        msg.data = text
        self.speech_publisher.publish(msg)

        return {
            'status': 'success',
            'text': text,
            'language': language
        }

    def _location_to_pose(self, location: str) -> PoseStamped:
        """Convert location string to PoseStamped message"""
        # This would use a map or semantic location database in practice
        location_map = {
            'kitchen': (1.0, 2.0, 0.0),
            'living_room': (5.0, 1.0, 0.0),
            'bedroom': (-2.0, 3.0, 0.0),
            'office': (0.0, -1.0, 0.0)
        }

        x, y, theta = location_map.get(location, (0.0, 0.0, 0.0))

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = theta

        return pose
```

## Safety and Validation

### Plan Validation System

```python
class PlanValidator:
    def __init__(self, cognitive_engine: LLMCognitiveEngine):
        self.engine = cognitive_engine
        self.safety_constraints = self._load_safety_constraints()

    def validate_plan(self, plan: List[PlanStep], context: Dict[str, Any]) -> Dict[str, Any]:
        """Validate a plan for safety and feasibility"""
        validation_results = {
            'is_valid': True,
            'warnings': [],
            'errors': [],
            'safety_score': 10.0
        }

        # Check each action in the plan
        for i, step in enumerate(plan):
            action = step.action

            # Check prerequisites
            missing_prereqs = self._check_prerequisites(action, context, plan[:i])
            if missing_prereqs:
                validation_results['errors'].append(
                    f"Action {i} ({action.name}) missing prerequisites: {missing_prereqs}"
                )
                validation_results['is_valid'] = False

            # Check safety constraints
            safety_violations = self._check_safety_constraints(action, context)
            if safety_violations:
                validation_results['warnings'].extend(safety_violations)
                validation_results['safety_score'] -= len(safety_violations) * 2

            # Check for potential conflicts with future actions
            conflicts = self._check_future_conflicts(action, plan[i+1:])
            if conflicts:
                validation_results['warnings'].append(
                    f"Action {i} ({action.name}) may conflict with future actions: {conflicts}"
                )

        return validation_results

    def _check_prerequisites(self, action: Action, context: Dict[str, Any], previous_actions: List[PlanStep]) -> List[str]:
        """Check if action prerequisites are met"""
        missing = []
        for prereq in action.prerequisites:
            # Check if prerequisite was achieved by previous actions
            prereq_met = any(
                prereq in step.action.effects for step in previous_actions
            )

            if not prereq_met:
                missing.append(prereq)

        return missing

    def _check_safety_constraints(self, action: Action, context: Dict[str, Any]) -> List[str]:
        """Check if action violates safety constraints"""
        violations = []

        # Check if action involves dangerous movements
        if action.name == 'move_to':
            location = action.parameters.get('location', '')
            if location in self.safety_constraints.get('forbidden_locations', []):
                violations.append(f"Moving to forbidden location: {location}")

        # Check if action parameters are within safe limits
        if action.name == 'move_to':
            speed = action.parameters.get('speed', 0.5)
            max_safe_speed = self.safety_constraints.get('max_safe_speed', 1.0)
            if speed > max_safe_speed:
                violations.append(f"Speed {speed} exceeds safe limit of {max_safe_speed}")

        return violations

    def _check_future_conflicts(self, action: Action, future_actions: List[PlanStep]) -> List[str]:
        """Check for conflicts between action and future actions"""
        conflicts = []

        # Check if action negates effects needed by future actions
        action_effects = set(action.effects)
        for future_action in future_actions:
            if action_effects.intersection(future_action.action.prerequisites):
                conflicts.append(future_action.action.name)

        return conflicts

    def _load_safety_constraints(self) -> Dict[str, Any]:
        """Load safety constraints from configuration"""
        return {
            'forbidden_locations': ['restricted_area', 'dangerous_zone'],
            'max_safe_speed': 0.8,
            'max_payload': 5.0,  # kg
            'forbidden_actions': ['dangerous_action']
        }
```

## Performance Optimization

### Caching and Optimization

```python
import functools
import hashlib
from typing import Tuple

class OptimizedLLMInterface:
    def __init__(self, cognitive_engine: LLMCognitiveEngine):
        self.engine = cognitive_engine
        self.response_cache = {}
        self.cache_size_limit = 100

    def plan_with_cache(self, goal: str, context: Dict[str, Any]) -> List[PlanStep]:
        """Plan with caching to avoid repeated LLM calls"""
        # Create cache key
        cache_key = self._create_cache_key(goal, context)

        # Check cache first
        if cache_key in self.response_cache:
            print("Using cached plan")
            return self.response_cache[cache_key]

        # Generate new plan
        plan = self.engine.plan_action_sequence(goal, context)

        # Cache the result
        self._add_to_cache(cache_key, plan)

        return plan

    def _create_cache_key(self, goal: str, context: Dict[str, Any]) -> str:
        """Create a cache key from goal and context"""
        context_str = json.dumps(context, sort_keys=True)
        combined = f"{goal}||{context_str}"
        return hashlib.md5(combined.encode()).hexdigest()

    def _add_to_cache(self, key: str, plan: List[PlanStep]):
        """Add plan to cache with size management"""
        if len(self.response_cache) >= self.cache_size_limit:
            # Remove oldest entry (this is simplified - in practice use OrderedDict)
            oldest_key = next(iter(self.response_cache))
            del self.response_cache[oldest_key]

        self.response_cache[key] = plan

    def batch_plan(self, goals: List[str], context: Dict[str, Any]) -> Dict[str, List[PlanStep]]:
        """Generate plans for multiple goals efficiently"""
        results = {}

        # Use threading for parallel processing (if supported by your LLM provider)
        import concurrent.futures

        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
            # Submit all planning tasks
            future_to_goal = {
                executor.submit(self.plan_with_cache, goal, context): goal
                for goal in goals
            }

            # Collect results
            for future in concurrent.futures.as_completed(future_to_goal):
                goal = future_to_goal[future]
                try:
                    plan = future.result()
                    results[goal] = plan
                except Exception as e:
                    print(f"Error planning for goal '{goal}': {e}")
                    results[goal] = []  # Empty plan on error

        return results
```

## LLM Action Planning Code Examples

### 1. Complete LLM Action Planning System

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
import openai
import json
import asyncio
import threading
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, asdict
from enum import Enum
import time

class ActionStatus(Enum):
    PENDING = "pending"
    EXECUTING = "executing"
    SUCCESS = "success"
    FAILED = "failed"
    CANCELLED = "cancelled"

@dataclass
class Action:
    """Represents an atomic action for the robot"""
    id: str
    name: str
    parameters: Dict[str, Any]
    description: str
    prerequisites: List[str]
    effects: List[str]

@dataclass
class PlanStep:
    """A step in the execution plan"""
    action: Action
    status: ActionStatus
    result: Optional[Dict[str, Any]] = None
    error: Optional[str] = None

class LLMActionPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_action_planner_node')

        # Initialize OpenAI API
        self.api_key = self.declare_parameter('openai_api_key', '').get_parameter_value().string_value
        if self.api_key:
            openai.api_key = self.api_key
        else:
            self.get_logger().warning("No OpenAI API key provided. Using mock responses.")

        # Initialize components
        self.context_manager = RobotContextManager()
        self.llm_interface = LLMInterface(self.api_key)
        self.plan_executor = PlanExecutor()

        # ROS publishers and subscribers
        self.command_sub = self.create_subscription(
            String, '/robot/command', self.command_callback, 10
        )
        self.status_pub = self.create_publisher(String, '/robot/action_status', 10)
        self.result_pub = self.create_publisher(String, '/robot/action_result', 10)

        # Action execution thread
        self.execution_thread = None
        self.current_plan = None
        self.execution_active = False

        self.get_logger().info('LLM Action Planner node initialized')

    def command_callback(self, msg):
        """Handle incoming commands from the voice pipeline or other sources"""
        command_text = msg.data
        self.get_logger().info(f'Received command: {command_text}')

        # Get robot context
        context = self.context_manager.get_current_context()

        # Generate plan using LLM
        plan = self.llm_interface.generate_plan(command_text, context)

        if plan:
            self.get_logger().info(f'Generated plan with {len(plan)} steps')

            # Execute the plan
            self.execute_plan(plan)
        else:
            self.get_logger().error('Failed to generate plan')

    def execute_plan(self, plan):
        """Execute the generated plan in a separate thread"""
        self.current_plan = plan
        self.execution_active = True

        if self.execution_thread and self.execution_thread.is_alive():
            self.execution_thread.join(timeout=2.0)

        self.execution_thread = threading.Thread(target=self._execute_plan_thread, args=(plan,))
        self.execution_thread.daemon = True
        self.execution_thread.start()

    def _execute_plan_thread(self, plan):
        """Execute plan in a separate thread"""
        for i, plan_step in enumerate(plan):
            if not self.execution_active:
                plan_step.status = ActionStatus.CANCELLED
                break

            self.get_logger().info(f'Executing step {i+1}/{len(plan)}: {plan_step.action.name}')

            # Execute the action
            result = self.plan_executor.execute_action(plan_step.action)

            if result['success']:
                plan_step.status = ActionStatus.SUCCESS
                plan_step.result = result
            else:
                plan_step.status = ActionStatus.FAILED
                plan_step.error = result.get('error', 'Unknown error')
                break  # Stop execution on failure

            # Publish status
            status_msg = String()
            status_msg.data = f"Step {i+1}: {plan_step.status.value} - {plan_step.action.name}"
            self.status_pub.publish(status_msg)

        # Publish final result
        result_msg = String()
        result_msg.data = f"Plan execution completed with {sum(1 for step in plan if step.status == ActionStatus.SUCCESS)}/{len(plan)} successes"
        self.result_pub.publish(result_msg)

    def cancel_execution(self):
        """Cancel current plan execution"""
        self.execution_active = False
        if self.execution_thread:
            self.execution_thread.join(timeout=2.0)

class RobotContextManager:
    """Manages robot state and context for LLM planning"""

    def __init__(self):
        self.robot_pose = Pose()
        self.joint_states = JointState()
        self.environment_objects = []
        self.task_history = []
        self.current_capabilities = []

    def get_current_context(self) -> Dict[str, Any]:
        """Get current robot context for planning"""
        return {
            "robot_pose": {
                "position": {
                    "x": self.robot_pose.position.x,
                    "y": self.robot_pose.position.y,
                    "z": self.robot_pose.position.z
                },
                "orientation": {
                    "x": self.robot_pose.orientation.x,
                    "y": self.robot_pose.orientation.y,
                    "z": self.robot_pose.orientation.z,
                    "w": self.robot_pose.orientation.w
                }
            },
            "joint_states": {
                "name": list(self.joint_states.name),
                "position": list(self.joint_states.position),
                "velocity": list(self.joint_states.velocity),
                "effort": list(self.joint_states.effort)
            },
            "environment_objects": self.environment_objects,
            "task_history": self.task_history,
            "capabilities": self.current_capabilities,
            "timestamp": time.time()
        }

class LLMInterface:
    """Interface to communicate with LLM for action planning"""

    def __init__(self, api_key: str):
        self.api_key = api_key
        if api_key:
            openai.api_key = api_key

    def generate_plan(self, command: str, context: Dict[str, Any]) -> Optional[List[PlanStep]]:
        """Generate an action plan using LLM"""
        if not self.api_key:
            # Mock implementation
            return self._generate_mock_plan(command)

        try:
            # Create a prompt for the LLM
            prompt = self._create_planning_prompt(command, context)

            # Call the LLM
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": "You are an action planner for a humanoid robot. Generate a sequence of actions to accomplish the user's goal. Respond with a JSON array of actions."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=1000
            )

            # Parse the response
            plan_json = response.choices[0].message.content

            # Convert to PlanStep objects
            plan_data = json.loads(plan_json)
            return self._convert_to_plan_steps(plan_data)

        except Exception as e:
            self.get_logger().error(f'Error generating plan: {str(e)}')
            return None

    def _create_planning_prompt(self, command: str, context: Dict[str, Any]) -> str:
        """Create a prompt for the LLM with context"""
        context_str = json.dumps(context, indent=2)

        return f"""
        Given the following robot context:
        {context_str}

        Generate a sequence of actions to accomplish this command:
        "{command}"

        Each action should have:
        - id: A unique identifier
        - name: The action name (e.g., "move_to_location", "pick_object", "speak")
        - parameters: A dictionary of parameters needed for the action
        - description: A brief description of what the action does
        - prerequisites: Any conditions that must be met before the action
        - effects: What changes the action makes to the world state

        Respond with a JSON array of actions. Keep the plan concise but complete.
        """

    def _generate_mock_plan(self, command: str) -> Optional[List[PlanStep]]:
        """Generate a mock plan for demonstration purposes"""
        # This is a simplified mock implementation
        if "move" in command.lower():
            action = Action(
                id="move_001",
                name="move_to_location",
                parameters={"x": 1.0, "y": 2.0, "theta": 0.0},
                description="Move to specified location",
                prerequisites=[],
                effects=["robot_position_changed"]
            )
            return [PlanStep(action=action, status=ActionStatus.PENDING)]
        elif "speak" in command.lower():
            action = Action(
                id="speak_001",
                name="speak_text",
                parameters={"text": "Hello, I am a humanoid robot"},
                description="Speak the specified text",
                prerequisites=[],
                effects=["spoken_text"]
            )
            return [PlanStep(action=action, status=ActionStatus.PENDING)]
        else:
            return []

    def _convert_to_plan_steps(self, plan_data: List[Dict]) -> List[PlanStep]:
        """Convert LLM response to PlanStep objects"""
        plan_steps = []
        for action_data in plan_data:
            action = Action(**action_data)
            plan_step = PlanStep(action=action, status=ActionStatus.PENDING)
            plan_steps.append(plan_step)
        return plan_steps

class PlanExecutor:
    """Executes the action plan on the robot"""

    def __init__(self):
        self.action_handlers = {
            "move_to_location": self._handle_move_to_location,
            "pick_object": self._handle_pick_object,
            "place_object": self._handle_place_object,
            "speak_text": self._handle_speak_text,
            "look_at": self._handle_look_at,
            "wave": self._handle_wave
        }

    def execute_action(self, action: Action) -> Dict[str, Any]:
        """Execute a single action"""
        handler = self.action_handlers.get(action.name)

        if not handler:
            return {
                "success": False,
                "error": f"No handler for action: {action.name}",
                "action_id": action.id
            }

        try:
            result = handler(action.parameters)
            return {
                "success": True,
                "result": result,
                "action_id": action.id
            }
        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "action_id": action.id
            }

    def _handle_move_to_location(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Handle move to location action"""
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        theta = params.get('theta', 0.0)

        # In a real implementation, this would interface with the navigation system
        # For now, we'll simulate the movement
        time.sleep(1.0)  # Simulate movement time

        return {
            "destination": {"x": x, "y": y, "theta": theta},
            "status": "arrived",
            "execution_time": 1.0
        }

    def _handle_pick_object(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Handle pick object action"""
        object_id = params.get('object_id', '')
        arm = params.get('arm', 'right')

        # In a real implementation, this would interface with the manipulation system
        # For now, we'll simulate the pick
        time.sleep(1.0)  # Simulate picking time

        return {
            "object_id": object_id,
            "arm": arm,
            "status": "picked",
            "execution_time": 1.0
        }

    def _handle_place_object(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Handle place object action"""
        object_id = params.get('object_id', '')
        location = params.get('location', {})

        # In a real implementation, this would interface with the manipulation system
        # For now, we'll simulate the placement
        time.sleep(1.0)  # Simulate placement time

        return {
            "object_id": object_id,
            "location": location,
            "status": "placed",
            "execution_time": 1.0
        }

    def _handle_speak_text(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Handle speak text action"""
        text = params.get('text', '')

        # In a real implementation, this would interface with the text-to-speech system
        # For now, we'll just log the text
        print(f"Speaking: {text}")

        return {
            "text": text,
            "status": "spoken",
            "execution_time": 0.5
        }

    def _handle_look_at(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Handle look at action"""
        target = params.get('target', {})

        # In a real implementation, this would interface with the head control system
        # For now, we'll simulate the look
        time.sleep(0.5)  # Simulate look time

        return {
            "target": target,
            "status": "looking",
            "execution_time": 0.5
        }

    def _handle_wave(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Handle wave action"""
        arm = params.get('arm', 'right')

        # In a real implementation, this would interface with the arm control system
        # For now, we'll simulate the wave
        time.sleep(1.0)  # Simulate wave time

        return {
            "arm": arm,
            "status": "waved",
            "execution_time": 1.0
        }

def main(args=None):
    rclpy.init(args=args)
    llm_planner = LLMActionPlannerNode()

    try:
        rclpy.spin(llm_planner)
    except KeyboardInterrupt:
        llm_planner.cancel_execution()
    finally:
        llm_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Advanced LLM Planning with Memory and Learning

```python
import pickle
from datetime import datetime
import os

class AdvancedLLMPlanner:
    """Advanced LLM planner with memory and learning capabilities"""

    def __init__(self, model_name="gpt-4"):
        self.model_name = model_name
        self.memory_db = {}  # Simple in-memory database
        self.execution_history = []
        self.known_patterns = {}  # Learned patterns
        self.performance_metrics = {}  # Track success rates

    def learn_from_execution(self, command: str, plan: List[PlanStep], outcome: str):
        """Learn from plan execution outcomes"""
        # Store the command-plan-outcome triplet
        key = hash(command)
        self.memory_db[key] = {
            "command": command,
            "plan": plan,
            "outcome": outcome,
            "timestamp": datetime.now(),
            "success": outcome == "success"
        }

        # Update performance metrics
        if command not in self.performance_metrics:
            self.performance_metrics[command] = {"attempts": 0, "successes": 0}

        self.performance_metrics[command]["attempts"] += 1
        if outcome == "success":
            self.performance_metrics[command]["successes"] += 1

    def predict_plan_success(self, command: str) -> float:
        """Predict the likelihood of plan success based on history"""
        if command in self.performance_metrics:
            metrics = self.performance_metrics[command]
            return metrics["successes"] / max(metrics["attempts"], 1)
        return 0.5  # Default 50% success rate for unknown commands

    def adapt_plan_for_command(self, command: str, base_plan: List[PlanStep]) -> List[PlanStep]:
        """Adapt a base plan based on learned patterns and context"""
        # Look for similar past commands
        for past_cmd, data in self.memory_db.items():
            if self._commands_similar(command, data["command"]):
                # Adapt the plan based on past successful variations
                return self._adapt_plan(base_plan, data["plan"], data["outcome"])

        return base_plan  # Return original plan if no adaptations found

    def _commands_similar(self, cmd1: str, cmd2: str) -> bool:
        """Check if two commands are semantically similar"""
        # Simple keyword-based similarity (in practice, use embeddings)
        keywords1 = set(cmd1.lower().split())
        keywords2 = set(cmd2.lower().split())

        # Calculate overlap
        intersection = keywords1.intersection(keywords2)
        union = keywords1.union(keywords2)

        if union:
            jaccard_similarity = len(intersection) / len(union)
            return jaccard_similarity > 0.5  # Threshold for similarity
        return False

    def _adapt_plan(self, base_plan: List[PlanStep], past_plan: List[PlanStep], outcome: str) -> List[PlanStep]:
        """Adapt a plan based on a similar past plan and its outcome"""
        # If the past plan succeeded, use it as reference
        # If it failed, modify based on failure patterns
        adapted_plan = []

        for i, base_step in enumerate(base_plan):
            if i < len(past_plan):
                past_step = past_plan[i]

                # If past execution failed at this step, modify parameters
                if outcome == "failure" and i == len(past_plan) - 1:  # Failed at this step
                    # Apply corrective modifications
                    modified_action = self._modify_action_for_failure(base_step.action, past_step.action)
                    adapted_plan.append(PlanStep(action=modified_action, status=ActionStatus.PENDING))
                else:
                    adapted_plan.append(base_step)
            else:
                adapted_plan.append(base_step)

        return adapted_plan

    def _modify_action_for_failure(self, base_action: Action, failed_action: Action) -> Action:
        """Modify an action based on how a similar action failed"""
        # This is a simplified implementation
        # In practice, you'd have more sophisticated failure analysis
        modified_params = base_action.parameters.copy()

        # Example: If movement failed due to obstacle, try alternative path
        if base_action.name == "move_to_location":
            # Add some variation to the destination
            modified_params["x"] += 0.1  # Small offset
            modified_params["y"] += 0.1  # Small offset

        return Action(
            id=base_action.id,
            name=base_action.name,
            parameters=modified_params,
            description=base_action.description,
            prerequisites=base_action.prerequisites,
            effects=base_action.effects
        )

class ContextualLLMPlanner:
    """LLM planner that incorporates rich contextual information"""

    def __init__(self):
        self.advanced_planner = AdvancedLLMPlanner()
        self.context_enricher = ContextEnricher()

    def generate_contextual_plan(self, command: str, raw_context: Dict[str, Any]) -> Optional[List[PlanStep]]:
        """Generate a plan with enriched context and learning"""
        # Enrich the context
        enriched_context = self.context_enricher.enrich(raw_context)

        # Predict success probability
        success_prob = self.advanced_planner.predict_plan_success(command)

        # Generate base plan
        base_plan = self._generate_base_plan(command, enriched_context)

        if base_plan:
            # Adapt plan based on learning
            adapted_plan = self.advanced_planner.adapt_plan_for_command(command, base_plan)
            return adapted_plan

        return base_plan

    def _generate_base_plan(self, command: str, context: Dict[str, Any]) -> Optional[List[PlanStep]]:
        """Generate a base plan using LLM (simplified implementation)"""
        # This would call the LLM in a real implementation
        # For now, returning a mock plan
        if "move" in command.lower():
            action = Action(
                id="move_001",
                name="move_to_location",
                parameters={"x": 1.0, "y": 2.0, "theta": 0.0},
                description="Move to specified location",
                prerequisites=[],
                effects=["robot_position_changed"]
            )
            return [PlanStep(action=action, status=ActionStatus.PENDING)]
        return []

class ContextEnricher:
    """Enriches context with additional information for planning"""

    def enrich(self, raw_context: Dict[str, Any]) -> Dict[str, Any]:
        """Add additional context information"""
        enriched = raw_context.copy()

        # Add temporal context
        enriched["time_of_day"] = datetime.now().strftime("%H:%M")
        enriched["day_of_week"] = datetime.now().strftime("%A")

        # Add environmental context
        enriched["lighting_conditions"] = self._estimate_lighting(raw_context)
        enriched["noise_level"] = self._estimate_noise(raw_context)

        # Add social context
        enriched["people_nearby"] = self._detect_people(raw_context)

        # Add robot state context
        enriched["battery_level"] = self._get_battery_level(raw_context)
        enriched["current_task"] = self._get_current_task(raw_context)

        return enriched

    def _estimate_lighting(self, context: Dict[str, Any]) -> str:
        """Estimate lighting conditions from context"""
        # In practice, this would analyze camera data or use light sensors
        return "normal"

    def _estimate_noise(self, context: Dict[str, Any]) -> str:
        """Estimate noise level from context"""
        # In practice, this would analyze microphone data
        return "quiet"

    def _detect_people(self, context: Dict[str, Any]) -> int:
        """Detect number of people nearby"""
        # In practice, this would analyze camera or LIDAR data
        return 1

    def _get_battery_level(self, context: Dict[str, Any]) -> float:
        """Get current battery level"""
        # In practice, this would read from robot's power system
        return 0.85  # 85%

    def _get_current_task(self, context: Dict[str, Any]) -> str:
        """Get current task if any"""
        return "idle"
```

### 3. LLM Planning Configuration and Deployment

```yaml
# LLM action planning configuration
llm_action_planner:
  ros__parameters:
    # OpenAI API configuration
    openai_api_key: ""
    openai_model: "gpt-4"  # or "gpt-3.5-turbo"
    openai_temperature: 0.3
    openai_max_tokens: 1000

    # Planning parameters
    max_plan_steps: 20
    plan_timeout: 30.0  # seconds
    retry_attempts: 3

    # Context management
    context_refresh_rate: 1.0  # seconds
    context_history_size: 100

    # Execution parameters
    execution_thread_pool_size: 3
    action_timeout: 10.0  # seconds per action
    safety_check_interval: 0.1  # seconds

    # Learning and adaptation
    memory_retention_hours: 24.0
    learning_enabled: true
    adaptation_threshold: 0.7  # minimum similarity for adaptation

    # Performance monitoring
    metrics_collection_enabled: true
    metrics_report_interval: 60.0  # seconds
    performance_threshold: 0.8  # minimum acceptable success rate
```

## Summary

LLM-based action planning provides humanoid robots with sophisticated cognitive capabilities, enabling them to interpret complex natural language commands and generate appropriate action sequences. The key components include a cognitive engine that interfaces with LLMs, context management for maintaining robot state, plan generation and execution systems, and safety validation mechanisms. By combining the reasoning power of LLMs with the physical capabilities of humanoid robots, we can create more intuitive and capable robotic systems that can adapt to complex, real-world environments and tasks.