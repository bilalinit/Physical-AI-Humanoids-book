---
sidebar_position: 5
title: 'Chapter 5: Physics in Gazebo'
description: 'Understanding physics simulation in Gazebo for humanoid robotics and Physical AI applications'
---

# Chapter 5: Physics in Gazebo

## Introduction

Gazebo is a powerful physics simulation environment that forms a crucial part of the robotics development pipeline. This chapter explores how to set up realistic physics simulations for humanoid robots, covering both basic simulation concepts and advanced techniques for creating physically accurate environments that bridge the gap between simulation and real-world robotics.

## Understanding Gazebo Physics Engine

Gazebo uses Open Dynamics Engine (ODE), Bullet, or Simbody as its underlying physics engine. Each engine has specific strengths:

- **ODE**: Fast, good for basic simulations
- **Bullet**: More accurate, handles complex contacts
- **Simbody**: Best for biomechanics and complex articulated systems

For humanoid robotics, Bullet is typically preferred due to its superior handling of contact dynamics and stability.

### Physics Configuration

The physics engine is configured in the world file:

```xml
<physics type="bullet">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
</physics>
```

For humanoid robots, smaller step sizes (0.001 or smaller) are recommended to maintain stability with complex joint configurations.

## Creating Physics Models for Humanoid Robots

### URDF Physics Properties

Physics properties are defined in the URDF file within `<collision>` and `<inertial>` tags:

```xml
<link name="thigh_link">
  <inertial>
    <mass value="2.5" />
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0"
             iyy="0.01" iyz="0.0" izz="0.02" />
  </inertial>

  <collision>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <geometry>
      <capsule length="0.3" radius="0.05"/>
    </geometry>
  </collision>

  <visual>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <geometry>
      <capsule length="0.3" radius="0.05"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
</link>
```

### Inertial Calculations

Accurate inertial properties are crucial for stable simulation:

- **Mass**: Should match real-world robot components
- **Center of Mass**: Critical for balance and stability
- **Inertia Tensor**: Affects how the robot responds to forces and torques

For humanoid limbs, approximate as cylinders or capsules:
- Thigh: m=2.5kg, length=0.3m, radius=0.05m
- Shank: m=1.5kg, length=0.3m, radius=0.04m
- Foot: m=0.8kg, box dimensions 0.15x0.08x0.05m

## Contact Dynamics and Friction

### Contact Parameters

Contact properties are defined in the `<gazebo>` tag for each link:

```xml
<gazebo reference="foot_link">
  <mu1>1.0</mu1>  <!-- Primary friction coefficient -->
  <mu2>1.0</mu2>  <!-- Secondary friction coefficient -->
  <kp>1000000.0</kp>  <!-- Contact stiffness -->
  <kd>100.0</kd>      <!-- Contact damping -->
  <max_vel>100.0</max_vel>
  <min_depth>0.001</min_depth>
</gazebo>
```

For humanoid robots walking on various surfaces:
- Indoor floors: μ=0.8-1.0
- Outdoor terrain: μ=0.6-0.9
- Slippery surfaces: μ=0.2-0.4

## Advanced Physics Concepts

### Joint Dynamics

Joint properties affect how the robot moves and interacts:

```xml
<joint name="hip_pitch" type="revolute">
  <parent link="torso_link"/>
  <child link="thigh_link"/>
  <origin xyz="0 0 -0.1" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="5"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

### Gazebo Plugins for Physics

Use plugins to enhance physics simulation:

```xml
<gazebo>
  <plugin name="ground_truth_odom" filename="libgazebo_ros_p3d.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>100.0</updateRate>
    <bodyName>base_link</bodyName>
    <topicName>ground_truth/state</topicName>
    <gaussianNoise>0.01</gaussianNoise>
  </plugin>
</gazebo>
```

### Additional Gazebo Plugin Examples

Here are several commonly used Gazebo plugins for humanoid robotics:

#### 1. IMU Sensor Plugin
```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

#### 2. Force-Torque Sensor Plugin
```xml
<gazebo reference="ft_sensor_joint">
  <sensor name="ft_sensor" type="force_torque">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
  </sensor>
</gazebo>
```

#### 3. Joint State Publisher Plugin
```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=joint_states</remapping>
    </ros>
    <update_rate>30</update_rate>
    <joint_name>hip_pitch</joint_name>
    <joint_name>knee_pitch</joint_name>
    <joint_name>ankle_pitch</joint_name>
    <joint_name>ankle_roll</joint_name>
  </plugin>
</gazebo>
```

#### 4. Joint Position Controller Plugin
```xml
<gazebo>
  <plugin name="position_controller" filename="libgazebo_ros_control.so">
    <robotNamespace>/robot</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    <legacyModeNS>true</legacyModeNS>
  </plugin>
</gazebo>
```

#### 5. Contact Sensor Plugin
```xml
<gazebo reference="foot_link">
  <sensor name="foot_contact" type="contact">
    <always_on>true</always_on>
    <update_rate>1000.0</update_rate>
    <contact>
      <collision>foot_link_collision</collision>
    </contact>
    <plugin name="contact_plugin" filename="libgazebo_ros_bumper.so">
      <alwaysOn>true</alwaysOn>
      <topicName>contact</topicName>
      <frameName>foot_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

#### 6. Camera Sensor Plugin
```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>30.0</updateRate>
      <cameraName>robot/camera1</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
      <hackBaseline>0.07</hackBaseline>
      <distortionK1>0.0</distortionK1>
      <distortionK2>0.0</distortionK2>
      <distortionK3>0.0</distortionK3>
      <distortionT1>0.0</distortionT1>
      <distortionT2>0.0</distortionT2>
    </plugin>
  </sensor>
</gazebo>
```

### Plugin Configuration Best Practices

When implementing Gazebo plugins for humanoid robots:

1. **Update Rates**: Match update rates to your control loop frequency (typically 100Hz for humanoid control)
2. **Noise Modeling**: Include realistic sensor noise to improve simulation-to-reality transfer
3. **Resource Management**: Limit update rates to maintain simulation performance
4. **Namespace Organization**: Use consistent namespaces for multi-robot simulations
5. **Error Handling**: Implement proper error handling in plugin code

### Creating Custom Plugins

For specialized humanoid applications, you may need to develop custom plugins:

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>

namespace gazebo
{
  class HumanoidBalancePlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the model pointer for convenience
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&HumanoidBalancePlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply balance control logic here
      // Access model properties: this->model->GetWorldPose()
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(HumanoidBalancePlugin)
}
```

These plugins enable sophisticated simulation of humanoid robot sensors, actuators, and control systems, providing realistic testing environments for complex robotic behaviors.

## Simulation Scenarios for Humanoid Robotics

### Walking Simulation

Create scenarios to test walking gaits:

1. **Flat ground walking**: Basic locomotion testing
2. **Slope walking**: Testing on inclined surfaces
3. **Stair climbing**: Complex multi-contact scenarios
4. **Obstacle navigation**: Stepping over obstacles

### Balance and Stability Testing

Physics simulation allows testing balance controllers:

```python
# Example balance controller
def balance_controller(robot_state, target_pose):
    # Calculate center of mass
    com = calculate_com(robot_state)

    # Calculate zero moment point
    zmp = calculate_zmp(com, robot_state.velocity)

    # Adjust foot placement based on ZMP
    if zmp.x > stability_threshold:
        adjust_foot_placement(robot_state)

    return control_commands
```

## Physics Parameter Tuning

### Step Size Optimization

- Start with 0.001s for humanoid robots
- Increase gradually while monitoring stability
- Monitor for energy drift and joint limit violations

### Solver Parameters

```xml
<physics type="bullet">
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Physics Validation

### Simulation-to-Reality Transfer

Key metrics for validating physics simulation:

1. **Kinematic accuracy**: Joint positions match between sim and real
2. **Dynamic behavior**: Balance and walking patterns similar
3. **Energy consumption**: Comparable to real robot
4. **Contact forces**: Realistic interaction forces

### Validation Techniques

- Compare trajectory tracking performance
- Validate contact detection and response
- Test controller robustness across simulation conditions

## Best Practices

1. **Start simple**: Begin with basic shapes and gradually add complexity
2. **Validate incrementally**: Test each component separately
3. **Match real hardware**: Use accurate mass and inertial properties
4. **Consider computational cost**: Balance accuracy with simulation speed
5. **Document parameters**: Keep records of validated physics settings

## Summary

Physics simulation in Gazebo is essential for developing and testing humanoid robots. Proper configuration of physics parameters, accurate inertial properties, and careful validation ensure that simulation results transfer effectively to real-world applications. With well-tuned physics parameters, Gazebo becomes a powerful tool for developing complex humanoid behaviors before deployment on physical hardware.