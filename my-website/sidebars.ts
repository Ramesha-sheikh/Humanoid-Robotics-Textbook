module.exports = {
  docs: [
    {
      type: 'category',
      label: '00 - Introduction to Physical AI',
      items: ['00-introduction/index'],
    },
    {
      type: 'category',
      label: 'Module 1 – ROS 2',
      items: ['01-module-1-ros2/01-ros2-introduction', '01-module-1-ros2/02-nodes-topics-services', '01-module-1-ros2/03-building-ros-packages', '01-module-1-ros2/04-urdf-xacro-humanoid', '01-module-1-ros2/05-launch-and-parameters'],
    },
    { type: 'category', label: 'Module 2 – Digital Twin', items: ['module-2-digital-twin/chapter-09', 'module-2-digital-twin/chapter-10', 'module-2-digital-twin/chapter-11'] },
    {
      type: 'category',
      label: 'Module 3 – NVIDIA Isaac Sim',
      items: ['03-module-3-isaac/01-isaac-sim-installation', '03-module-3-isaac/02-synthetic-data-and-vslam', '03-module-3-isaac/03-nav2-bipedal-locomotion', '03-module-3-isaac/04-reinforcement-learning-humanoid'],
    },
    {
      type: 'category',
      label: 'Module 4 – VLA & Capstone',
      items: ['04-module-4-vla-capstone/01-whisper-voice-commands', '04-module-4-vla-capstone/02-llm-to-ros-planning', '04-module-4-vla-capstone/03-multi-modal-integration', '04-module-4-vla-capstone/04-capstone-autonomous-humanoid'],
    },
    { type: 'category', label: 'Appendices', items: ['appendices/installation', 'appendices/sim-to-real'] },
  ],
};