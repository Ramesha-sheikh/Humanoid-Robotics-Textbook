import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';
import { FaRobot, FaCubes, FaGamepad } from 'react-icons/fa';

type FeatureItem = {
  title: string;
  Icon: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Comprehensive Introduction to ROS2',
    Icon: FaRobot,
    description: (
      <>
        Dive deep into the Robot Operating System 2 (ROS2), the industry-standard framework for robotics. Learn the core concepts, from nodes and topics to services and actions, and build a solid foundation for developing complex robot behaviors.
      </>
    ),
  },
  {
    title: 'Build Your Own Digital Twin',
    Icon: FaCubes,
    description: (
      <>
        Create a detailed virtual model of a humanoid robot from scratch. Master the use of URDF and Xacro to define the robot's physical properties, and learn how to assemble a complete digital twin for simulation and testing.
      </>
    ),
  },
  {
    title: 'Simulate and Control',
    Icon: FaGamepad,
    description: (
      <>
        Bring your robot to life in a simulated environment. Use Gazebo to test and refine your robot's movements and interactions. Learn how to develop and implement control strategies in ROS2 to make your humanoid robot walk, grasp, and perform tasks.
      </>
    ),
  },
];

function Feature({title, Icon, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4', styles.feature)}>
      <div className="text--center">
        <Icon className={styles.featureIcon} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
        <div className="row">
          <div className={clsx('col col--12', styles.buttonContainer)}>
            <Link
              className="button button--secondary button--lg"
              to="/docs/introduction">
              Read the Book
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}
