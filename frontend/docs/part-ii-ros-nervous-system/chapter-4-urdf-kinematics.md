---
sidebar_position: 3
title: 'Chapter 4: URDF & Kinematics'
---

# Chapter 4: URDF & Kinematics

## Learning Objectives

By the end of this chapter, you should be able to:
- Create and validate Unified Robot Description Format (URDF) files
- Understand forward and inverse kinematics for humanoid robots
- Implement kinematic chains for 12-DOF bipedal robots
- Use KDL and MoveIt! for kinematic computations
- Generate Xacro macros for complex robot descriptions

## Prerequisites

Before starting this chapter, you should:
- Complete Chapter 3: ROS 2 Architecture
- Basic understanding of linear algebra and transformation matrices
- Familiarity with XML syntax

## Introduction

The Unified Robot Description Format (URDF) is the standard way to describe robots in ROS, defining their physical and kinematic properties. For humanoid robots, URDF is crucial for simulation, visualization, and kinematic computations. This chapter covers creating detailed robot descriptions and understanding kinematic principles for humanoid robotics.

## URDF Fundamentals

### URDF Structure

URDF is an XML format that describes robot structure through links and joints:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- First link connected via joint -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </visual>
  </link>

  <!-- Joint connecting the links -->
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

### Links

Links represent rigid bodies in the robot. Each link can have:
- Visual properties (for visualization)
- Collision properties (for physics simulation)
- Inertial properties (for dynamics)

### Joints

Joints define the connection between links. Joint types include:
- **revolute**: Rotational joint with limits
- **continuous**: Continuous rotational joint (like a wheel)
- **prismatic**: Linear sliding joint
- **fixed**: Rigid connection (no movement)
- **floating**: 6-DOF connection
- **planar**: Movement in a plane

## Humanoid Robot URDF

### 12-DOF Bipedal Robot Structure

A typical 12-DOF bipedal humanoid has:
- 6 DOF per leg (hip: 3 DOF, knee: 1 DOF, ankle: 2 DOF)
- Often simplified to 6 DOF total for basic walking

```xml
<?xml version="1.0"?>
<robot name="bipedal_robot">
  <!-- Base/Pelvis link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Left leg -->
  <link name="left_hip">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.03"/>
      </geometry>
      <origin xyz="0 0 -0.05" rpy="1.57 0 0"/>
    </visual>
  </link>

  <joint name="left_hip_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="0 0.1 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.025"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="1.57 0 0"/>
    </visual>
  </link>

  <joint name="left_knee" type="revolute">
    <parent link="left_hip"/>
    <child link="left_thigh"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_shin">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.025"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="1.57 0 0"/>
    </visual>
  </link>

  <joint name="left_ankle" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <!-- Similar structure for right leg -->
  <link name="right_hip">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.03"/>
      </geometry>
      <origin xyz="0 0 -0.05" rpy="1.57 0 0"/>
    </visual>
  </link>

  <joint name="right_hip_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="right_hip"/>
    <origin xyz="0 -0.1 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <link name="right_thigh">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.025"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="1.57 0 0"/>
    </visual>
  </link>

  <joint name="right_knee" type="revolute">
    <parent link="right_hip"/>
    <child link="right_thigh"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_shin">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.025"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="1.57 0 0"/>
    </visual>
  </link>

  <joint name="right_ankle" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Xacro for Complex Robots

Xacro (XML Macros) allows parameterization and reuse in URDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="bipedal_xacro">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="leg_length" value="0.3" />
  <xacro:property name="hip_offset" value="0.1" />

  <!-- Macro for leg -->
  <xacro:macro name="leg" params="side reflect">
    <link name="${side}_hip">
      <visual>
        <geometry>
          <cylinder length="0.1" radius="0.03"/>
        </geometry>
        <origin xyz="0 0 -0.05" rpy="1.57 0 0"/>
      </visual>
    </link>

    <joint name="${side}_hip_yaw" type="revolute">
      <parent link="base_link"/>
      <child link="${side}_hip"/>
      <origin xyz="0 ${reflect * hip_offset} 0" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
    </joint>

    <link name="${side}_thigh">
      <visual>
        <geometry>
          <cylinder length="${leg_length}" radius="0.025"/>
        </geometry>
        <origin xyz="0 0 -${leg_length/2}" rpy="1.57 0 0"/>
      </visual>
    </link>

    <joint name="${side}_knee" type="revolute">
      <parent link="${side}_hip"/>
      <child link="${side}_thigh"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="1.57" effort="100" velocity="1"/>
    </joint>

    <link name="${side}_shin">
      <visual>
        <geometry>
          <cylinder length="${leg_length}" radius="0.025"/>
        </geometry>
        <origin xyz="0 0 -${leg_length/2}" rpy="1.57 0 0"/>
      </visual>
    </link>

    <joint name="${side}_ankle" type="revolute">
      <parent link="${side}_thigh"/>
      <child link="${side}_shin"/>
      <origin xyz="0 0 -${leg_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Instantiate legs using macro -->
  <xacro:leg side="left" reflect="1" />
  <xacro:leg side="right" reflect="-1" />

</robot>
```

## Kinematics Fundamentals

### Forward Kinematics

Forward kinematics computes the end-effector position from joint angles. For humanoid robots, this is essential for understanding where limbs are in space.

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

def dh_transform(a, alpha, d, theta):
    """Denavit-Hartenberg transformation matrix"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    T = np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])
    return T

def forward_kinematics(joint_angles, dh_params):
    """Compute forward kinematics for a chain"""
    T_total = np.eye(4)

    for i, (a, alpha, d, _) in enumerate(dh_params):
        theta = joint_angles[i]
        T_link = dh_transform(a, alpha, d, theta)
        T_total = T_total @ T_link

    return T_total

# Example: 3-DOF leg segment
dh_params = [
    (0, np.pi/2, 0, 0),      # Hip yaw
    (0, 0, 0.3, 0),         # Knee
    (0, 0, 0.3, 0)          # Ankle
]

joint_angles = [0.1, 0.5, -0.2]  # Radians
end_effector_pose = forward_kinematics(joint_angles, dh_params)
print(f"End effector pose:\n{end_effector_pose}")
```

### Inverse Kinematics

Inverse kinematics computes joint angles needed to achieve a desired end-effector position. For humanoid robots, this is crucial for walking, reaching, and balancing.

```python
import numpy as np
from scipy.optimize import minimize

def inverse_kinematics(target_pos, initial_joints, dh_params, link_lengths):
    """Solve inverse kinematics using optimization"""

    def objective(joint_angles):
        # Forward kinematics to get current position
        current_pos = forward_kinematics(joint_angles, dh_params)[:3, 3]
        # Return distance to target
        return np.linalg.norm(current_pos - target_pos)

    result = minimize(objective, initial_joints, method='BFGS')
    return result.x

# Example usage
target_position = np.array([0.4, 0.1, -0.2])  # x, y, z
initial_guess = [0.0, 0.0, 0.0]
solution = inverse_kinematics(target_position, initial_guess, dh_params, [0.3, 0.3])
print(f"Joint angles for target: {solution}")
```

## Using KDL for Kinematics

The Kinematics and Dynamics Library (KDL) provides efficient kinematic computations:

```python
import PyKDL as kdl

def create_chain():
    """Create a KDL chain for a leg"""
    chain = kdl.Chain()

    # Add segments for each joint
    chain.addSegment(kdl.Segment(
        kdl.Joint(kdl.Joint.RotZ),  # Hip yaw
        kdl.Frame(kdl.Vector(0, 0, 0))
    ))

    chain.addSegment(kdl.Segment(
        kdl.Joint(kdl.Joint.RotY),  # Hip pitch
        kdl.Frame(kdl.Vector(0, 0, -0.1))
    ))

    chain.addSegment(kdl.Segment(
        kdl.Joint(kdl.Joint.RotY),  # Knee
        kdl.Frame(kdl.Vector(0, 0, -0.3))
    ))

    chain.addSegment(kdl.Segment(
        kdl.Joint(kdl.Joint.RotY),  # Ankle
        kdl.Frame(kdl.Vector(0, 0, -0.3))
    ))

    return chain

def compute_kinematics(chain, joint_angles):
    """Compute forward kinematics using KDL"""
    fk_solver = kdl.ChainFkSolverPos_recursive(chain)

    # Create joint array
    joints = kdl.JntArray(len(joint_angles))
    for i, angle in enumerate(joint_angles):
        joints[i] = angle

    # Compute forward kinematics
    end_frame = kdl.Frame()
    result = fk_solver.JntToCart(joints, end_frame)

    if result >= 0:
        # Extract position and orientation
        pos = end_frame.p
        rot = end_frame.M
        return [pos[0], pos[1], pos[2]], rot
    else:
        return None, None
```

## MoveIt! Integration

MoveIt! provides advanced motion planning and kinematic capabilities:

```python
import moveit_commander
import rospy
from geometry_msgs.msg import Pose

class HumanoidMoveGroup:
    def __init__(self):
        # Initialize MoveIt!
        moveit_commander.roscpp_initialize(sys.argv)

        # Robot interface
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # Move group for legs
        self.left_leg_group = moveit_commander.MoveGroupCommander("left_leg")
        self.right_leg_group = moveit_commander.MoveGroupCommander("right_leg")

    def move_to_pose(self, group, target_pose):
        """Move end effector to target pose"""
        group.set_pose_target(target_pose)
        plan = group.plan()

        if plan[0]:  # If plan is valid
            group.execute(plan[1], wait=True)
            group.stop()
            group.clear_pose_targets()
            return True
        return False

    def move_to_joint_values(self, group, joint_values):
        """Move joints to specific values"""
        group.set_joint_value_target(joint_values)
        plan = group.plan()

        if plan[0]:
            group.execute(plan[1], wait=True)
            group.stop()
            return True
        return False
```

## Kinematic Constraints for Humanoid Robots

Humanoid robots have specific kinematic constraints that must be respected:

```python
def check_balance_constraint(robot_state, com_limits):
    """Check if robot's center of mass is within balance limits"""
    # Calculate center of mass based on joint angles
    com = calculate_center_of_mass(robot_state)

    # Check if within support polygon (based on foot positions)
    left_foot_pos = get_foot_position(robot_state, 'left')
    right_foot_pos = get_foot_position(robot_state, 'right')

    # Define support polygon
    support_polygon = create_support_polygon(left_foot_pos, right_foot_pos)

    return point_in_polygon(com[:2], support_polygon)

def calculate_center_of_mass(robot_state):
    """Calculate robot's center of mass"""
    total_mass = 0
    weighted_pos = np.array([0.0, 0.0, 0.0])

    for link_name, mass in robot_state.link_masses.items():
        link_pose = get_link_pose(robot_state, link_name)
        weighted_pos += mass * np.array([link_pose.x, link_pose.y, link_pose.z])
        total_mass += mass

    return weighted_pos / total_mass
```

## URDF Validation and Testing

### Validating URDF Files

```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# Visualize robot in RViz
ros2 run rviz2 rviz2

# Launch robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat robot.urdf)
```

### Testing Kinematic Chains

```python
import unittest
from urdf_parser_py.urdf import URDF

class URDFValidator(unittest.TestCase):
    def test_urdf_load(self):
        """Test that URDF file loads correctly"""
        robot = URDF.from_xml_file('path/to/robot.urdf')
        self.assertIsNotNone(robot)
        self.assertGreater(len(robot.links), 0)
        self.assertGreater(len(robot.joints), 0)

    def test_kinematic_chain(self):
        """Test that kinematic chain is valid"""
        robot = URDF.from_xml_file('path/to/robot.urdf')

        # Check that all joints connect existing links
        for joint in robot.joints:
            self.assertIn(joint.parent, [l.name for l in robot.links])
            self.assertIn(joint.child, [l.name for l in robot.links])
```

## GPU-Accelerated Kinematics

For humanoid robots with many DOF, GPU acceleration can speed up kinematic computations:

```python
import torch

class GPUKinematics:
    def __init__(self, device='cuda'):
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')

    def forward_kinematics_batch(self, joint_angles_batch):
        """Compute FK for multiple poses simultaneously"""
        # Convert to GPU tensors
        angles = torch.tensor(joint_angles_batch, device=self.device)

        # Perform batch computation using GPU
        # This is a simplified example - real implementation would use
        # GPU-optimized transformation matrices
        results = self._compute_batch_fk(angles)

        return results.cpu().numpy()

    def _compute_batch_fk(self, angles):
        """Internal batch FK computation"""
        # Implementation would use PyTorch operations
        # for parallel transformation computation
        pass
```

## Summary

URDF and kinematics form the foundation for representing and controlling humanoid robots in ROS. Understanding how to properly define robot structure and solve kinematic problems is essential for humanoid robotics applications.

## Next Steps

Now that you understand URDF and kinematics, continue to [Chapter 5: Physics in Gazebo](/docs/part-iii-digital-twin/chapter-5-physics-gazebo) to learn about physics simulation for humanoid robots.

## Exercises

1. Create a URDF file for a simple 6-DOF humanoid leg
2. Implement forward kinematics for a 3-DOF planar manipulator
3. Use MoveIt! to plan and execute a simple movement
4. Create a Xacro macro for a humanoid arm with 7 DOF