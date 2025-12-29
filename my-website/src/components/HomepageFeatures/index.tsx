import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';
import { FaRobot, FaCubes, FaGamepad, FaBrain, FaCode, FaBook, FaRocket, FaMicrochip, FaEye, FaCog } from 'react-icons/fa';

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

const CourseModules = [
  {
    title: 'Module 1: ROS 2 Basics',
    icon: FaCode,
    description: 'Master nodes, topics, services, and URDF for humanoid robotics'
  },
  {
    title: 'Module 2: Digital Twin',
    icon: FaCubes,
    description: 'Build and simulate robots in Gazebo with physics and sensors'
  },
  {
    title: 'Module 3: NVIDIA Isaac Sim',
    icon: FaMicrochip,
    description: 'Advanced perception, RL, and navigation with Isaac Sim'
  },
  {
    title: 'Module 4: VLA Integration',
    icon: FaBrain,
    description: 'Voice commands and LLM-based action planning'
  },
];

const Stats = [
  { number: '4+', label: 'Modules' },
  { number: '30+', label: 'Lessons' },
  { number: '100+', label: 'Code Examples' },
  { number: '24/7', label: 'AI Chatbot Support' },
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

function StatsSection(): ReactNode {
  return (
    <section className={styles.statsSection}>
      <div className="container">
        <div className={styles.statsGrid}>
          {Stats.map((stat, idx) => (
            <div key={idx} className={styles.statCard}>
              <div className={styles.statNumber}>{stat.number}</div>
              <div className={styles.statLabel}>{stat.label}</div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CourseModulesSection(): ReactNode {
  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            <FaBook className={styles.titleIcon} />
            Course Curriculum
          </Heading>
          <p className={styles.sectionSubtitle}>
            A complete journey from basics to advanced humanoid robotics
          </p>
        </div>
        <div className={styles.modulesGrid}>
          {CourseModules.map((module, idx) => (
            <div key={idx} className={styles.moduleCard}>
              <div className={styles.moduleIcon}>
                <module.icon />
              </div>
              <h3 className={styles.moduleTitle}>{module.title}</h3>
              <p className={styles.moduleDescription}>{module.description}</p>
              <div className={styles.moduleNumber}>0{idx + 1}</div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function WhyChooseSection(): ReactNode {
  return (
    <section className={styles.whyChooseSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            <FaRocket className={styles.titleIcon} />
            Why Choose This Course?
          </Heading>
        </div>
        <div className={styles.benefitsGrid}>
          <div className={styles.benefitCard}>
            <FaCode className={styles.benefitIcon} />
            <h3>Hands-On Projects</h3>
            <p>Build real-world robots with practical coding exercises and simulations</p>
          </div>
          <div className={styles.benefitCard}>
            <FaBrain className={styles.benefitIcon} />
            <h3>AI-Powered Learning</h3>
            <p>Get instant help from our 24/7 RAG chatbot trained on course content</p>
          </div>
          <div className={styles.benefitCard}>
            <FaEye className={styles.benefitIcon} />
            <h3>Industry Standard Tools</h3>
            <p>Learn ROS2, Gazebo, Isaac Sim, and modern robotics frameworks</p>
          </div>
          <div className={styles.benefitCard}>
            <FaCog className={styles.benefitIcon} />
            <h3>Personalized Experience</h3>
            <p>Content adapted to your programming and robotics background</p>
          </div>
        </div>
      </div>
    </section>
  );
}

function CTASection(): ReactNode {
  return (
    <section className={styles.ctaSection}>
      <div className="container">
        <div className={styles.ctaContent}>
          <Heading as="h2" className={styles.ctaTitle}>
            Ready to Build Humanoid Robots?
          </Heading>
          <p className={styles.ctaSubtitle}>
            Start your journey into the future of robotics and AI
          </p>
          <div className={styles.ctaButtons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/introduction">
              Start Learning Now
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/docs/module-1-ros2/ros2-introduction">
              Explore Modules
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <>
      {/* Stats Section */}
      <StatsSection />

      {/* Main Features */}
      <section className={styles.features}>
        <div className="container">
          <div className="row">
            {FeatureList.map((props, idx) => (
              <Feature key={idx} {...props} />
            ))}
          </div>
        </div>
      </section>

      {/* Course Modules */}
      <CourseModulesSection />

      {/* Why Choose */}
      <WhyChooseSection />

      {/* Call to Action */}
      <CTASection />
    </>
  );
}
