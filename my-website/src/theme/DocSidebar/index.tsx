import React, { type ReactNode } from 'react';
import DocSidebar from '@theme-original/DocSidebar';
import type DocSidebarType from '@theme/DocSidebar';
import type { WrapperProps } from '@docusaurus/types';
import { useAuth } from '../../components/Auth/AuthContext';
import styles from './styles.module.css';

type Props = WrapperProps<typeof DocSidebarType>;

/**
 * Personalized sidebar that highlights relevant content based on user background
 */
export default function DocSidebarWrapper(props: Props): ReactNode {
  const { user, isAuthenticated } = useAuth();

  // If not authenticated, show default sidebar
  if (!isAuthenticated || !user) {
    return <DocSidebar {...props} />;
  }

  // Get user's learning preferences
  const learningGoal = user.learning_goal || 'General Learning';
  const programmingExp = user.programming_experience || 'None';
  const rosExp = user.ros_experience || 'None';
  const roboticsProjects = user.robotics_projects || 'None';

  // Determine experience level
  const isBeginnerLevel =
    (rosExp === 'None' || rosExp === null) &&
    (roboticsProjects === 'None' || roboticsProjects === null);

  // Create personalized recommendations
  const recommendations: string[] = [];

  // Based on Learning Goal
  if (learningGoal === 'Robot Manipulation') {
    recommendations.push('Module 3: Isaac Sim - Perception & RL for manipulation');
    recommendations.push('Module 2: Digital Twin - Simulate manipulation tasks');
  } else if (learningGoal === 'Humanoid Locomotion') {
    recommendations.push('Module 3: Isaac Sim - Navigation & locomotion');
    recommendations.push('Module 2: Digital Twin - Physics simulation');
  } else if (learningGoal === 'Computer Vision') {
    recommendations.push('Module 3: Isaac Sim - Perception pipeline');
    recommendations.push('Introduction - Sensor systems');
  } else if (learningGoal === 'Control Systems') {
    recommendations.push('Module 2: Digital Twin - Physics simulation');
    recommendations.push('Module 1: ROS 2 - Services & Actions');
  } else if (learningGoal === 'ROS Development') {
    recommendations.push('Module 1: ROS 2 Basics - Core concepts');
    recommendations.push('Module 1: ROS 2 - Architecture & nodes');
  }

  // Based on Experience Level
  if (isBeginnerLevel) {
    recommendations.push('Start with Introduction and Module 1');
  } else {
    recommendations.push('Advanced: Jump to Module 3 for Isaac Sim');
  }

  // Based on Programming Experience
  if (programmingExp === 'None' || programmingExp === null) {
    recommendations.push('Python basics covered in ROS 2 module');
  }

  return (
    <>
      {/* Personalized Recommendations Banner */}
      <div className={styles.recommendationsPanel}>
        <div className={styles.recommendationsHeader}>
          <span className={styles.sparkleIcon}>✨</span>
          <h3>Recommended for You</h3>
        </div>

        <div className={styles.userProfile}>
          <p className={styles.profileLabel}>
            <strong>Goal:</strong> {learningGoal}
          </p>
          {programmingExp && programmingExp !== 'None' && (
            <p className={styles.profileLabel}>
              <strong>Programming:</strong> {programmingExp}
            </p>
          )}
          {rosExp && rosExp !== 'None' && (
            <p className={styles.profileLabel}>
              <strong>ROS:</strong> {rosExp}
            </p>
          )}
        </div>

        <div className={styles.recommendationsList}>
          {recommendations.slice(0, 3).map((rec, index) => (
            <div key={index} className={styles.recommendationItem}>
              <span className={styles.checkIcon}>→</span>
              <span>{rec}</span>
            </div>
          ))}
        </div>

        <div className={styles.levelBadge}>
          {isBeginnerLevel ? (
            <span className={styles.beginnerBadge}>🌱 Beginner Path</span>
          ) : (
            <span className={styles.advancedBadge}>🚀 Advanced Path</span>
          )}
        </div>
      </div>

      {/* Original Sidebar */}
      <DocSidebar {...props} />
    </>
  );
}
