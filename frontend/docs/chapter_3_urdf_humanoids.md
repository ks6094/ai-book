# Understanding URDF for Humanoids

## What is URDF?

URDF stands for Unified Robot Description Format. It's an XML-based format used in ROS to describe robot models, including their physical and visual properties. URDF allows you to define the structure of a robot in terms of links (rigid parts) and joints (connections between links that may move).

### Purpose of URDF

URDF serves several critical purposes in robotics:

1. **Robot Modeling**: Define the physical structure of a robot including all links and joints
2. **Visualization**: Provide information needed to visualize the robot in tools like RViz
3. **Simulation**: Supply physical properties needed for robot simulation
4. **Kinematics**: Enable kinematic calculations for robot motion planning
5. **Collision Detection**: Define collision properties for safety and planning

### URDF Components

A URDF file typically contains:

- **Links**: Rigid parts of the robot (e.g., base, arms, legs)
- **Joints**: Connections between links that define how they can move relative to each other
- **Visual**: Information about how the link appears visually
- **Collision**: Information about the collision properties of the link
- **Inertial**: Physical properties like mass and moment of inertia

## Defining Robot Models

### Basic URDF Structure

Here's the basic structure of a URDF file:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Define links -->
  <link name="link_name">
    <visual>
      <!-- Visual properties -->
    </visual>
    <collision>
      <!-- Collision properties -->
    </collision>
    <inertial>
      <!-- Inertial properties -->
    </inertial>
  </link>

  <!-- Define joints -->
  <joint name="joint_name" type="joint_type">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <!-- Joint properties -->
  </joint>
</robot>
```

### Link Definition

A link represents a rigid part of the robot. Here's a detailed example:

```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### Joint Definition

Joints define how links can move relative to each other. Common joint types include:

- **fixed**: No movement between links
- **revolute**: Rotational movement around an axis
- **prismatic**: Linear movement along an axis
- **continuous**: Continuous rotational movement (like a revolute joint without limits)
- **floating**: 6-DOF movement (rarely used)

Example joint definition:

```xml
<joint name="joint_name" type="revolute">
  <parent link="base_link"/>
  <child link="upper_arm_link"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-2.0" upper="1.0" effort="30" velocity="1.0"/>
</joint>
```

## Creating Humanoid Robot Models

### Humanoid Robot Structure

A humanoid robot typically has a structure like this:

```
base_link (torso)
├── head_link
├── left_upper_arm_link
│   ├── left_lower_arm_link
│   └── left_hand_link
├── right_upper_arm_link
│   ├── right_lower_arm_link
│   └── right_hand_link
├── left_upper_leg_link
│   ├── left_lower_leg_link
│   └── left_foot_link
└── right_upper_leg_link
    ├── right_lower_leg_link
    └── right_foot_link
```

### Complete Humanoid URDF Example

Here's a simplified example of a humanoid robot model:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <cylinder length="0.8" radius="0.15"/>
      </geometry>
      <origin xyz="0 0 0.4"/>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.8" radius="0.15"/>
      </geometry>
      <origin xyz="0 0 0.4"/>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0.1"/>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0.1"/>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.8"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="1.57079632679 0 0"/>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="1.57079632679 0 0"/>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.1 0.15 0.6"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

### Best Practices for Humanoid URDF

When creating humanoid robot models:

1. **Proper Mass Distribution**: Assign realistic masses to each link
2. **Accurate Dimensions**: Use actual robot dimensions for accurate simulation
3. **Consistent Naming**: Use a consistent naming convention for links and joints
4. **Joint Limits**: Set appropriate limits for joint movement
5. **Inertial Properties**: Calculate or estimate accurate inertial properties
6. **Collision vs Visual**: Use simpler geometries for collision than visual when possible for performance

## Visualizing and Testing URDF

### Using RViz for Visualization

RViz is ROS's 3D visualization tool that can display URDF models:

1. Launch RViz
2. Add a RobotModel display
3. Set the Robot Description parameter to your URDF parameter name
4. The robot model should appear in the visualization

### Using Gazebo for Simulation

Gazebo is a physics simulator that can load URDF models:

1. Convert your URDF to SDF format (Gazebo's native format) or use directly
2. Create a Gazebo world file
3. Launch Gazebo with your robot model

### Testing URDF Validity

Before using your URDF, validate it:

1. **XML Validation**: Ensure the XML is well-formed
2. **URDF Validation**: Use tools like `check_urdf` to validate the structure
3. **Kinematic Chains**: Verify that all joints form proper kinematic chains
4. **No Dangling Joints**: Ensure all joints connect to existing links

### Common Issues and Solutions

- **Missing Links**: Ensure every joint references existing parent and child links
- **Inertial Issues**: If simulation is unstable, check inertial properties
- **Joint Direction**: Verify joint axes are correctly oriented
- **Collision Issues**: Make sure collision meshes don't intersect in the default configuration

### Creating URDF with Xacro

For complex robots, use Xacro (XML Macros) to make URDF more manageable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <xacro:macro name="simple_cylinder" params="name radius length xyz *origin *material">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
        <xacro:insert_block name="origin"/>
        <xacro:insert_block name="material"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
        <xacro:insert_block name="origin"/>
      </collision>
      <inertial>
        <mass value="${radius * radius * length * 10}"/>
        <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
      </inertial>
    </link>
  </xacro:macro>

  <xacro:simple_cylinder name="base_link" radius="0.1" length="0.5" xyz="0 0 0">
    <origin xyz="0 0 0"/>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </xacro:simple_cylinder>
</robot>
```