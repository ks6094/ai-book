// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Course Overview',
      items: [
        'course-overview'
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'chapter_1_ros2_overview',
        'chapter_2_rclpy_python_agents',
        'chapter_3_urdf_humanoids'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'chapter_1_digital_twin_overview',
        'chapter_2_gazebo_physics_simulation',
        'chapter_3_unity_high_fidelity_simulation',
        'chapter_4_sensor_simulation'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'chapter_1_ai_robot_brain_overview',
        'chapter_2_isaac_sim_photorealistic_simulation',
        'chapter_3_isaac_ros_accelerated_perception',
        'chapter_4_navigation_and_path_planning'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'chapter_1_vla_overview',
        'chapter_2_voice_to_action',
        'chapter_3_cognitive_planning',
        'chapter_4_capstone_autonomous_humanoid'
      ],
    },
  ],
};

module.exports = sidebars;