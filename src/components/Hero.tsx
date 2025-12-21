import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Translate from '@docusaurus/Translate';
import Link from '@docusaurus/Link';
import clsx from 'clsx';
import styles from './Hero.module.css';

const HeroContent = () => {
  const { siteConfig } = useDocusaurusContext();

  return (
    <div className={styles.heroSection}>
      <div className={styles.heroContainer}>
        <div className={styles.imageContainer}>
          <img
            src={require('@site/static/img/hero-robot.jpg').default}
            alt="Robotics and AI"
            className={styles.heroImage}
          />
        </div>

        <div className={styles.textContent}>
          <h1 className={styles.heroTitle}>
            <Translate id="homepage.hero.title">
              Physical AI & Humanoid Robotics
            </Translate>
          </h1>

          <div className={styles.taglineContainer}>
            <p className={styles.heroSubtitle}>
              <Translate id="homepage.hero.tagline">
                A comprehensive AI-native textbook for embodied intelligence
              </Translate>
            </p>
          </div>

          <div className={styles.description}>
            <p>
              <Translate id="homepage.hero.description">
                A comprehensive AI-native textbook for embodied intelligence and robotics.
                Learn to build intelligent machines that can perceive, reason, and act in the physical world.
              </Translate>
            </p>
          </div>

          <div className={styles.buttons}>
            <Link
              className={clsx(
                'button button--secondary button--lg',
                styles.heroButton,
              )}
              to="/intro"
            >
              <Translate id="homepage.hero.startLearning">Start Learning</Translate>
            </Link>
            <Link
              className={clsx(
                'button button--outline button--lg',
                styles.heroSecondaryButton,
              )}
              to="/ros2/intro"
            >
              <Translate id="homepage.hero.exploreModules">Explore Modules</Translate>
            </Link>
          </div>
        </div>
      </div>
    </div>
  );
};

const CourseInfo = () => (
  <section className={styles.courseInfo}>
    <div className="container">
      <div className="row">
        <div className="col col--12">
          <h2 className={styles.sectionTitle}>
            <Translate id="homepage.courseInfo.title">About This Course</Translate>
          </h2>
          <p className={styles.sectionContent}>
            <Translate id="homepage.courseInfo.description">
              This comprehensive 13-week textbook covers the cutting-edge intersection of artificial intelligence and robotics. You'll learn to build embodied AI systems that can perceive, reason, and act in the physical world using state-of-the-art technologies including ROS 2, NVIDIA Isaac, Gazebo/Unity simulation, and Vision-Language-Action models.
            </Translate>
          </p>
        </div>
      </div>
    </div>
  </section>
);

const CourseModules = () => (
  <section className={styles.courseModules}>
    <div className="container">
      <div className="row">
        <div className="col col--12">
          <h2 className={styles.sectionTitle}>
            <Translate id="homepage.modules.title">Course Modules</Translate>
          </h2>
        </div>
      </div>
      <div className="row">
        <div className="col col--6">
          <div className={styles.moduleCard}>
            <div className={styles.emoji}>🤖</div>
            <h3><Translate id="homepage.modules.ros2.title">Module 1: ROS 2 - Robotic Nervous System</Translate></h3>
            <p><Translate id="homepage.modules.ros2.weeks">Weeks 1-3</Translate></p>
            <p><Translate id="homepage.modules.ros2.description">Learn the fundamentals of Robot Operating System 2, the nervous system of modern robotics. Covers nodes, topics, services, and actions.</Translate></p>
          </div>
        </div>
        <div className="col col--6">
          <div className={styles.moduleCard}>
            <div className={styles.emoji}>🕹️</div>
            <h3><Translate id="homepage.modules.gazebo.title">Module 2: Gazebo/Unity - Digital Twin</Translate></h3>
            <p><Translate id="homepage.modules.gazebo.weeks">Weeks 4-6</Translate></p>
            <p><Translate id="homepage.modules.gazebo.description">Explore simulation environments for robotics development. Learn to create digital twins for safe testing and validation.</Translate></p>
          </div>
        </div>
      </div>
      <div className="row">
        <div className="col col--6">
          <div className={styles.moduleCard}>
            <div className={styles.emoji}>🧠</div>
            <h3><Translate id="homepage.modules.isaac.title">Module 3: NVIDIA Isaac - AI Robot Brains</Translate></h3>
            <p><Translate id="homepage.modules.isaac.weeks">Weeks 7-9</Translate></p>
            <p><Translate id="homepage.modules.isaac.description">Discover AI-powered perception and decision-making for robots. Learn to integrate NVIDIA Isaac for vision, perception, and control.</Translate></p>
          </div>
        </div>
        <div className="col col--6">
          <div className={styles.moduleCard}>
            <div className={styles.emoji}>👁️</div>
            <h3><Translate id="homepage.modules.vla.title">Module 4: VLA - Vision-Language-Action</Translate></h3>
            <p><Translate id="homepage.modules.vla.weeks">Weeks 10-13</Translate></p>
            <p><Translate id="homepage.modules.vla.description">Master the integration of vision, language, and action for embodied intelligence. Learn how robots understand and respond to natural commands.</Translate></p>
          </div>
        </div>
      </div>
    </div>
  </section>
);

const CourseFeatures = () => (
  <section className={styles.courseFeatures}>
    <div className="container">
      <div className="row">
        <div className="col col--12">
          <h2 className={styles.sectionTitle}>
            <Translate id="homepage.features.title">Course Features</Translate>
          </h2>
        </div>
      </div>
      <div className="row">
        <div className="col col--6">
          <div className={styles.featureCard}>
            <div className={styles.emoji}>🤖</div>
            <h3><Translate id="homepage.features.aiNative.title">AI-Native Content</Translate></h3>
            <p><Translate id="homepage.features.aiNative.description">Content generated and enhanced with AI tools, ensuring up-to-date and comprehensive coverage of cutting-edge technologies.</Translate></p>
          </div>
        </div>
        <div className="col col--6">
          <div className={styles.featureCard}>
            <div className={styles.emoji}>🧪</div>
            <h3><Translate id="homepage.features.labs.title">Hands-On Labs</Translate></h3>
            <p><Translate id="homepage.features.labs.description">Practical lab exercises that connect theoretical concepts with real implementations on simulation and physical platforms.</Translate></p>
          </div>
        </div>
      </div>
      <div className="row">
        <div className="col col--6">
          <div className={styles.featureCard}>
            <div className={styles.emoji}>🏢</div>
            <h3><Translate id="homepage.features.industry.title">Industry Technologies</Translate></h3>
            <p><Translate id="homepage.features.industry.description">Learn with the same tools used in robotics research and industry: ROS 2, NVIDIA Isaac, Gazebo, Unity, and more.</Translate></p>
          </div>
        </div>
        <div className="col col--6">
          <div className={styles.featureCard}>
            <div className={styles.emoji}>🧩</div>
            <h3><Translate id="homepage.features.modular.title">Modular Design</Translate></h3>
            <p><Translate id="homepage.features.modular.description">Structured content organized by weeks and modules, making it easy to follow and adapt to different course schedules.</Translate></p>
          </div>
        </div>
      </div>
    </div>
  </section>
);

const QuickAccess = () => (
  <section className={styles.quickAccess}>
    <div className="container">
      <div className="row">
        <div className="col col--12">
          <h2 className={styles.sectionTitle}>
            <Translate id="homepage.quickAccess.title">Quick Access</Translate>
          </h2>
        </div>
      </div>
      <div className="row">
        <div className="col col--3">
          <div className={styles.accessCard}>
            <div className={styles.emoji}>🔬</div>
            <h3><Translate id="homepage.quickAccess.labs.title">Lab Exercises</Translate></h3>
            <p><Translate id="homepage.quickAccess.labs.description">Practical exercises for each module</Translate></p>
          </div>
        </div>
        <div className="col col--3">
          <div className={styles.accessCard}>
            <div className={styles.emoji}>📝</div>
            <h3><Translate id="homepage.quickAccess.assessments.title">Assessments</Translate></h3>
            <p><Translate id="homepage.quickAccess.assessments.description">Quizzes and assignments for each week</Translate></p>
          </div>
        </div>
        <div className="col col--3">
          <div className={styles.accessCard}>
            <div className={styles.emoji}>⚙️</div>
            <h3><Translate id="homepage.quickAccess.hardware.title">Hardware Setup</Translate></h3>
            <p><Translate id="homepage.quickAccess.hardware.description">Guides for configuring required equipment</Translate></p>
          </div>
        </div>
        <div className="col col--3">
          <div className={styles.accessCard}>
            <div className={styles.emoji}>🎓</div>
            <h3><Translate id="homepage.quickAccess.instructor.title">Instructor Guide</Translate></h3>
            <p><Translate id="homepage.quickAccess.instructor.description">Resources for educators using this textbook</Translate></p>
          </div>
        </div>
      </div>
    </div>
  </section>
);

const Hero = () => {
  return (
    <BrowserOnly fallback={<div>Loading hero section...</div>}>
      {() => (
        <>
          <HeroContent />
          <CourseInfo />
          <CourseModules />
          <CourseFeatures />
          <QuickAccess />
        </>
      )}
    </BrowserOnly>
  );
};

export default Hero;
