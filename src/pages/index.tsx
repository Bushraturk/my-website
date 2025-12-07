import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="row">
          <div className="col col--6">
            <h1 className="hero__title">{siteConfig.title}</h1>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <p className={styles.heroSubtitle}>
              A comprehensive AI-native textbook for embodied intelligence and robotics
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/physical-ai-textbook/intro">
                Start Learning - 13 Week Course
              </Link>
            </div>
          </div>
          <div className="col col--6">
            <div className={styles.heroGraphic}>
              <svg viewBox="0 0 400 300" className={styles.robotIllustration}>
                <rect x="150" y="100" width="100" height="100" rx="10" fill="#4c78a8" />
                <circle cx="200" cy="80" r="30" fill="#54c4d0" />
                <rect x="170" y="200" width="20" height="40" fill="#4c78a8" />
                <rect x="210" y="200" width="20" height="40" fill="#4c78a8" />
                <circle cx="180" cy="130" r="8" fill="#fff" />
                <circle cx="220" cy="130" r="8" fill="#fff" />
                <path d="M190 160 Q200 170 210 160" stroke="#fff" strokeWidth="3" fill="none" />
                <circle cx="100" cy="150" r="40" fill="#8e6c97" />
                <path d="M100 110 L100 190 M60 150 L140 150" stroke="#fff" strokeWidth="2" />
                <rect x="280" y="140" width="60" height="20" fill="#f0c050" />
                <text x="310" y="155" textAnchor="middle" fontSize="10" fill="#333" fontWeight="bold">GPU</text>
              </svg>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function CourseOverview() {
  return (
    <section className={styles.courseOverview}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('margin-bottom--lg', styles.sectionTitle)}>
              About This Course
            </Heading>
            <p className={styles.justifiedText}>
              This comprehensive 13-week textbook covers the cutting-edge intersection of artificial intelligence and robotics. 
              You'll learn to build embodied AI systems that can perceive, reason, and act in the physical world using state-of-the-art 
              technologies including ROS 2, NVIDIA Isaac, Gazebo/Unity simulation, and Vision-Language-Action models.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

function ModuleCards() {
  const modules = [
    {
      title: 'Module 1: ROS 2 - Robotic Nervous System',
      description: 'Learn the fundamentals of Robot Operating System 2, the nervous system of modern robotics. Covers nodes, topics, services, and actions.',
      weeks: 'Weeks 1-3',
      color: '#4c78a8',
      icon: '🤖',
    },
    {
      title: 'Module 2: Gazebo/Unity - Digital Twin',
      description: 'Explore simulation environments for robotics development. Learn to create digital twins for safe testing and validation.',
      weeks: 'Weeks 4-6',
      color: '#72b7c0',
      icon: '🕹️',
    },
    {
      title: 'Module 3: NVIDIA Isaac - AI Robot Brains',
      description: 'Discover AI-powered perception and decision-making for robots. Learn to integrate NVIDIA Isaac for vision, perception, and control.',
      weeks: 'Weeks 7-9',
      color: '#f58518',
      icon: '🧠',
    },
    {
      title: 'Module 4: VLA - Vision-Language-Action',
      description: 'Master the integration of vision, language, and action for embodied intelligence. Learn how robots understand and respond to natural commands.',
      weeks: 'Weeks 10-13',
      color: '#b279a2',
      icon: '👁️',
    },
  ];

  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('margin-bottom--lg', styles.sectionTitle)}>
              Course Modules
            </Heading>
          </div>
        </div>
        <div className="row">
          {modules.map((module, idx) => (
            <div key={idx} className="col col--3">
              <div className={styles.moduleCard} style={{borderLeftColor: module.color}}>
                <div className={styles.moduleIcon}>{module.icon}</div>
                <h3 className={styles.moduleTitle}>{module.title}</h3>
                <p className={styles.moduleWeeks}>{module.weeks}</p>
                <p className={styles.moduleDescription}>{module.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CourseFeatures() {
  const features = [
    {
      title: 'AI-Native Content',
      description: 'Content generated and enhanced with AI tools, ensuring up-to-date and comprehensive coverage of cutting-edge technologies.',
      icon: '🤖',
    },
    {
      title: 'Hands-On Labs',
      description: 'Practical lab exercises that connect theoretical concepts with real implementations on simulation and physical platforms.',
      icon: '🧪',
    },
    {
      title: 'Industry Technologies',
      description: 'Learn with the same tools used in robotics research and industry: ROS 2, NVIDIA Isaac, Gazebo, Unity, and more.',
      icon: '🏢',
    },
    {
      title: 'Modular Design',
      description: 'Structured content organized by weeks and modules, making it easy to follow and adapt to different course schedules.',
      icon: '🧩',
    },
  ];
  
  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('margin-bottom--lg', styles.sectionTitle)}>
              Course Features
            </Heading>
          </div>
        </div>
        <div className="row">
          {features.map((feature, idx) => (
            <div key={idx} className="col col--3">
              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>{feature.icon}</div>
                <h3 className={styles.featureTitle}>{feature.title}</h3>
                <p className={styles.featureDescription}>{feature.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function QuickAccess() {
  const resources = [
    {
      title: 'Lab Exercises',
      description: 'Practical exercises for each module',
      to: '/docs/ros2/lab-exercises/lab1',
      icon: '🔬',
    },
    {
      title: 'Assessments',
      description: 'Quizzes and assignments for each week',
      to: '/docs/ros2/assessments/quiz1',
      icon: '📝',
    },
    {
      title: 'Hardware Setup',
      description: 'Guides for configuring required equipment',
      to: '/docs/instructor-guide/offline-access',
      icon: '⚙️',
    },
    {
      title: 'Instructor Guide',
      description: 'Resources for educators using this textbook',
      to: '/docs/instructor-guide/intro',
      icon: '🎓',
    },
  ];

  return (
    <section className={styles.quickAccessSection}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('margin-bottom--lg', styles.sectionTitle)}>
              Quick Access
            </Heading>
          </div>
        </div>
        <div className="row">
          {resources.map((resource, idx) => (
            <div key={idx} className="col col--3">
              <div className={styles.resourceCard}>
                <div className={styles.resourceIcon}>{resource.icon}</div>
                <h3 className={styles.resourceTitle}>{resource.title}</h3>
                <p className={styles.resourceDescription}>{resource.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Robotics`}
      description="A comprehensive AI-native textbook for embodied intelligence and robotics">
      <HomepageHeader />
      <main>
        <CourseOverview />
        <ModuleCards />
        <CourseFeatures />
        <QuickAccess />
      </main>
    </Layout>
  );
}