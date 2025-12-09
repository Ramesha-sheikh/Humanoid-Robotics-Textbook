<<<<<<< HEAD
module.exports = {
  docs: [
    {
      type: 'category',
      label: '00 - Introduction to Physical AI',
      items: ['introduction/index'],
    },
    {
      type: 'category',
      label: 'Module 1 – ROS 2',
      items: ['module-1-ros2/ros2-introduction', 'module-1-ros2/nodes-topics-services', 'module-1-ros2/building-ros-packages', 'module-1-ros2/urdf-xacro-humanoid', 'module-1-ros2/launch-and-parameters'],
    },
    { type: 'category', label: 'Module 2 – Digital Twin', items: ['module-2-digital-twin/chapter-09', 'module-2-digital-twin/chapter-10', 'module-2-digital-twin/chapter-11'] },
    {
      type: 'category',
      label: 'Module 3 – NVIDIA Isaac Sim',
      items: ['module-3-isaac/isaac-sim-installation', 'module-3-isaac/synthetic-data-and-vslam', 'module-3-isaac/nav2-bipedal-locomotion', 'module-3-isaac/reinforcement-learning-humanoid'],
    },
    {
      type: 'category',
      label: 'Module 4 – VLA & Capstone',
      items: ['module-4-vla-capstone/whisper-voice-commands', 'module-4-vla-capstone/llm-to-ros-planning', 'module-4-vla-capstone/multi-modal-integration', 'module-4-vla-capstone/capstone-autonomous-humanoid'],
    },
    { type: 'category', label: 'Appendices', items: ['appendices/installation', 'appendices/sim-to-real'] },
  ],
};
=======
import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Week 1: Introduction to Physical AI',
      items: [
        'week1/foundations',
        'week1/embodied-intelligence',
        'week1/sensor-systems',
      ],
      link: {
        type: 'doc',
        id: 'week1/index',
      },
    },
    {
      type: 'category',
      label: 'Week 2: Robot Anatomy, Kinematics, and Control Basics',
      items: [
        'week2/robot-anatomy-actuators',
        'week2/robot-kinematics-intro',
        'week2/control-systems-basics',
      ],
      link: {
        type: 'doc',
        id: 'week2/index',
      },
    },
    {
      type: 'category',
      label: 'Week 3: ROS 2 Architecture & Core Concepts',
      items: [
        'week3/index',
        'week3/ros2-nodes',
        'week3/ros2-topics',
      ],
      link: {
        type: 'doc',
        id: 'week3/index',
      },
    },
    {
      type: 'category',
      label: 'Week 4: Advanced ROS 2 Topics & Communication',
      items: [
        'week4/advanced-ros2-topics',
        'week4/ros2-services',
      ],
      link: {
        type: 'doc',
        id: 'week4/index',
      },
    },
    {
      type: 'category',
      label: 'Week 5: Advanced ROS 2 Services & Actions',
      items: [
        'week5/ros2-services-deeper-dive',
        'week5/ros2-actions-comprehensive',
      ],
      link: {
        type: 'doc',
        id: 'week5/index',
      },
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
>>>>>>> a7d920193ce792bf5b2c8e211d93c297f7161419
