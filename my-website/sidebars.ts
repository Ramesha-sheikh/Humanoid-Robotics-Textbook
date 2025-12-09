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