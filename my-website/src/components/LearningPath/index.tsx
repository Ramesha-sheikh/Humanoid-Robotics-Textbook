import React, { useState } from 'react';
import { useAuth } from '../Auth/AuthContext';
import styles from './styles.module.css';

interface PathStep {
  title: string;
  description: string;
  modules: string[];
  estimatedTime: string;
  difficulty: 'Beginner' | 'Intermediate' | 'Advanced';
  icon: string;
}

/**
 * Personalized Learning Path Component
 * Shows recommended learning sequence based on user's background and goals
 */
export default function LearningPath(): JSX.Element {
  const { user, isAuthenticated } = useAuth();
  const [expandedStep, setExpandedStep] = useState<number | null>(0);

  if (!isAuthenticated || !user) {
    return (
      <div className={styles.notAuthenticatedMessage}>
        <p>Sign in to see your personalized learning path</p>
      </div>
    );
  }

  // Get user preferences
  const learningGoal = user.learning_goal || 'General Learning';
  const programmingExp = user.programming_experience || 'None';
  const rosExp = user.ros_experience || 'None';
  const roboticsProjects = user.robotics_projects || 'None';

  // Determine if user is beginner
  const isBeginner =
    (rosExp === 'None' || rosExp === null) &&
    (roboticsProjects === 'None' || roboticsProjects === null);

  // Define learning paths based on goals
  const getLearningPath = (): PathStep[] => {
    // Common foundation for all beginners
    const foundationSteps: PathStep[] = isBeginner
      ? [
          {
            title: 'Foundation: Introduction to Robotics',
            description: 'Start with robotics fundamentals and sensor systems',
            modules: [
              'Introduction - Physical AI Overview',
              'Introduction - Sensor Systems',
            ],
            estimatedTime: '2-3 hours',
            difficulty: 'Beginner',
            icon: '📚',
          },
          {
            title: 'ROS 2 Fundamentals',
            description: 'Learn ROS 2 basics - the industry standard for robotics',
            modules: [
              'Module 1: ROS 2 Introduction',
              'Module 1: Nodes & Topics',
              'Module 1: Services & Actions',
              'Module 1: URDF for Humanoids',
            ],
            estimatedTime: '8-10 hours',
            difficulty: 'Beginner',
            icon: '🤖',
          },
        ]
      : [];

    // Goal-specific paths
    if (learningGoal === 'Robot Manipulation') {
      return [
        ...foundationSteps,
        {
          title: 'Simulation & Digital Twin',
          description: 'Set up simulation environments for manipulation tasks',
          modules: [
            'Module 2: Gazebo Setup',
            'Module 2: Physics Simulation',
            'Module 2: Sensor Simulation',
          ],
          estimatedTime: '6-8 hours',
          difficulty: 'Intermediate',
          icon: '🎮',
        },
        {
          title: 'Advanced Manipulation with Isaac Sim',
          description: 'Perception, grasping, and reinforcement learning for manipulation',
          modules: [
            'Module 3: Isaac Sim Overview',
            'Module 3: Perception Pipeline',
            'Module 3: Reinforcement Learning',
          ],
          estimatedTime: '10-12 hours',
          difficulty: 'Advanced',
          icon: '🦾',
        },
        {
          title: 'VLA Integration for Manipulation',
          description: 'Use vision-language-action models for intelligent manipulation',
          modules: [
            'Module 4: Whisper for Voice Commands',
            'Module 4: LLM ROS Action Planner',
          ],
          estimatedTime: '6-8 hours',
          difficulty: 'Advanced',
          icon: '🧠',
        },
      ];
    } else if (learningGoal === 'Humanoid Locomotion') {
      return [
        ...foundationSteps,
        {
          title: 'Physics & Dynamics Simulation',
          description: 'Understand physics engines and locomotion simulation',
          modules: [
            'Module 2: Gazebo Setup',
            'Module 2: Physics Simulation',
          ],
          estimatedTime: '6-8 hours',
          difficulty: 'Intermediate',
          icon: '⚡',
        },
        {
          title: 'Locomotion with Isaac Sim',
          description: 'Navigation, pathfinding, and gait control',
          modules: [
            'Module 3: Isaac Sim Overview',
            'Module 3: Navigation with Nav2',
            'Module 3: Reinforcement Learning',
          ],
          estimatedTime: '10-12 hours',
          difficulty: 'Advanced',
          icon: '🚶',
        },
      ];
    } else if (learningGoal === 'Computer Vision') {
      return [
        ...foundationSteps,
        {
          title: 'Sensor Systems & Perception',
          description: 'Camera systems, depth sensors, and point clouds',
          modules: [
            'Introduction: Sensor Systems',
            'Module 2: Sensor Simulation',
          ],
          estimatedTime: '5-7 hours',
          difficulty: 'Intermediate',
          icon: '👁️',
        },
        {
          title: 'Perception Pipeline with Isaac Sim',
          description: 'Object detection, segmentation, and 3D perception',
          modules: [
            'Module 3: Isaac Sim Overview',
            'Module 3: Perception Pipeline',
          ],
          estimatedTime: '8-10 hours',
          difficulty: 'Advanced',
          icon: '📷',
        },
      ];
    } else if (learningGoal === 'ROS Development') {
      return [
        {
          title: 'ROS 2 Core Concepts',
          description: 'Master ROS 2 architecture and communication patterns',
          modules: [
            'Module 1: ROS 2 Introduction',
            'Module 1: ROS 2 Architecture',
            'Module 1: Nodes & Topics',
            'Module 1: Services & Actions',
          ],
          estimatedTime: '8-10 hours',
          difficulty: isBeginner ? 'Beginner' : 'Intermediate',
          icon: '🛠️',
        },
        {
          title: 'URDF & Robot Description',
          description: 'Model humanoid robots with URDF',
          modules: ['Module 1: URDF for Humanoids', 'Module 1: RCLPy Integration'],
          estimatedTime: '5-6 hours',
          difficulty: 'Intermediate',
          icon: '🤖',
        },
        {
          title: 'Simulation Integration',
          description: 'Connect ROS 2 with simulation environments',
          modules: [
            'Module 2: Gazebo Setup',
            'Module 3: Isaac Sim Overview',
          ],
          estimatedTime: '6-8 hours',
          difficulty: 'Intermediate',
          icon: '🔗',
        },
      ];
    } else {
      // General Learning path
      return [
        ...foundationSteps,
        {
          title: 'Simulation Environments',
          description: 'Learn Gazebo and digital twin concepts',
          modules: [
            'Module 2: Gazebo Setup',
            'Module 2: Physics Simulation',
          ],
          estimatedTime: '6-8 hours',
          difficulty: 'Intermediate',
          icon: '🎮',
        },
        {
          title: 'Advanced Tools with Isaac Sim',
          description: 'Explore NVIDIA Isaac Sim for advanced robotics',
          modules: [
            'Module 3: Isaac Sim Overview',
            'Module 3: Perception Pipeline',
            'Module 3: Navigation with Nav2',
          ],
          estimatedTime: '10-12 hours',
          difficulty: 'Advanced',
          icon: '🚀',
        },
      ];
    }
  };

  const learningPath = getLearningPath();

  return (
    <div className={styles.learningPathContainer}>
      <div className={styles.header}>
        <h2 className={styles.title}>
          <span className={styles.pathIcon}>🎯</span>
          Your Personalized Learning Path
        </h2>
        <p className={styles.subtitle}>
          Curated for: <strong>{learningGoal}</strong>
          {programmingExp && programmingExp !== 'None' && (
            <> | Programming: <strong>{programmingExp}</strong></>
          )}
        </p>
      </div>

      <div className={styles.pathSteps}>
        {learningPath.map((step, index) => (
          <div
            key={index}
            className={`${styles.step} ${
              expandedStep === index ? styles.expanded : ''
            }`}
          >
            <div
              className={styles.stepHeader}
              onClick={() => setExpandedStep(expandedStep === index ? null : index)}
            >
              <div className={styles.stepNumber}>
                <span className={styles.stepIcon}>{step.icon}</span>
                <span className={styles.numberBadge}>{index + 1}</span>
              </div>

              <div className={styles.stepInfo}>
                <h3 className={styles.stepTitle}>{step.title}</h3>
                <p className={styles.stepDescription}>{step.description}</p>
              </div>

              <div className={styles.stepMeta}>
                <span className={`${styles.difficultyBadge} ${styles[step.difficulty.toLowerCase()]}`}>
                  {step.difficulty}
                </span>
                <span className={styles.timeBadge}>⏱️ {step.estimatedTime}</span>
              </div>

              <span className={styles.expandArrow}>
                {expandedStep === index ? '▲' : '▼'}
              </span>
            </div>

            {expandedStep === index && (
              <div className={styles.stepContent}>
                <h4 className={styles.modulesHeader}>📖 Modules to Complete:</h4>
                <ul className={styles.modulesList}>
                  {step.modules.map((module, moduleIndex) => (
                    <li key={moduleIndex} className={styles.moduleItem}>
                      <span className={styles.checkboxIcon}>☐</span>
                      {module}
                    </li>
                  ))}
                </ul>
              </div>
            )}
          </div>
        ))}
      </div>

      <div className={styles.footer}>
        <p className={styles.footerText}>
          💡 <strong>Pro Tip:</strong> You can update your learning preferences in Profile Settings
          to see different recommendations.
        </p>
      </div>
    </div>
  );
}
