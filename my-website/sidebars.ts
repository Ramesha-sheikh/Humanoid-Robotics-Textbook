import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: [
        'introduction/introduction',
        'introduction/physical-ai-overview',
        'introduction/sensor-systems',
      ],
      link: {
        type: 'doc',
        id: 'introduction/introduction',
      },
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Basics',
      items: [
        'module-1-ros2/ros2-introduction',
        'module-1-ros2/nodes',
        'module-1-ros2/topics',
        'module-1-ros2/services',
        'module-1-ros2/urdf',
        'module-1-ros2/rclpy-integration',
        'module-1-ros2/ros2-architecture',
        'module-1-ros2/nodes-topics',
        'module-1-ros2/services-actions',
        'module-1-ros2/urdf-for-humanoids',
      ],
      link: {
        type: 'doc',
        id: 'module-1-ros2/ros2-introduction',
      },
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin & Simulation',
      items: [
        'module-2-digital-twin/gazebo-setup',
        'module-2-digital-twin/physics-simulation',
        'module-2-digital-twin/sensor-simulation',
        'module-2-digital-twin/unity-visualization',
        'module-2-digital-twin/index',
      ],
      link: {
        type: 'doc',
        id: 'module-2-digital-twin/gazebo-setup',
      },
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Sim',
      items: [
        'module-3-isaac/isaac-sim-overview',
        'module-3-isaac/perception-pipeline',
        'module-3-isaac/reinforcement-learning',
        'module-3-isaac/navigation-nav2',
        'module-3-isaac/index',
      ],
      link: {
        type: 'doc',
        id: 'module-3-isaac/isaac-sim-overview',
      },
    },
    {
      type: 'category',
      label: 'Module 4: VLA Capstone',
      items: [
        'module-4-vla/whisper-for-commands',
        'module-4-vla/llm-ros-action-planner',
        'module-4-vla/index',
      ],
      link: {
        type: 'doc',
        id: 'module-4-vla/whisper-for-commands',
      },
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/end-to-end-robot-pipeline',
        'capstone/index',
      ],
      link: {
        type: 'doc',
        id: 'capstone/end-to-end-robot-pipeline',
      },
    },
  ],
};

export default sidebars;
