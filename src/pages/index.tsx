import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Hero from '@site/src/components/Hero';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function WhoThisIsFor(): JSX.Element {
  return (
    <section className={styles.whoSection}>
      <div className={styles.sectionContainer}>
        <h2 className={styles.sectionTitle}>Who This Is For</h2>
        <div className={styles.audienceGrid}>
          <div className={styles.audienceCard}>
            <h3>AI/ML Engineers</h3>
            <p>
              Transition from virtual environments to physical robotics. Apply your deep learning
              expertise to embodied AI systems that interact with the real world.
            </p>
          </div>
          <div className={styles.audienceCard}>
            <h3>Robotics Developers</h3>
            <p>
              Level up from basic control systems to AI-powered autonomy. Integrate cutting-edge
              vision-language models into your robotic applications.
            </p>
          </div>
          <div className={styles.audienceCard}>
            <h3>Graduate Students & Researchers</h3>
            <p>
              Bridge the gap between academic research and practical implementation. Build complete
              systems from simulation to hardware deployment.
            </p>
          </div>
        </div>
        <div className={styles.prerequisites}>
          <h3>Prerequisites</h3>
          <ul>
            <li>Basic Python programming and Linux command line</li>
            <li>Familiarity with machine learning concepts (neural networks, training)</li>
            <li>No prior robotics experience required—we start from fundamentals</li>
          </ul>
        </div>
      </div>
    </section>
  );
}

function WhatYouBuild(): JSX.Element {
  return (
    <section className={styles.buildSection}>
      <div className={styles.sectionContainer}>
        <h2 className={styles.sectionTitle}>What You'll Build</h2>
        <div className={styles.projectsList}>
          <div className={styles.projectCard}>
            <div className={styles.projectNumber}>01</div>
            <h3>ROS 2 Navigation Stack</h3>
            <p>
              Build a complete autonomous navigation system with SLAM, path planning, and obstacle
              avoidance using ROS 2 and Nav2.
            </p>
          </div>
          <div className={styles.projectCard}>
            <div className={styles.projectNumber}>02</div>
            <h3>Gazebo Simulation Environment</h3>
            <p>
              Create photorealistic simulation worlds for testing robot behaviors before hardware
              deployment, including sensor models and physics.
            </p>
          </div>
          <div className={styles.projectCard}>
            <div className={styles.projectNumber}>03</div>
            <h3>Vision-Guided Manipulation</h3>
            <p>
              Implement computer vision pipelines in Isaac Sim for object detection, pose
              estimation, and grasp planning with robotic arms.
            </p>
          </div>
          <div className={styles.projectCard}>
            <div className={styles.projectNumber}>04</div>
            <h3>VLA-Powered Humanoid Agent</h3>
            <p>
              Train a vision-language-action model to control a humanoid robot through natural
              language commands and visual feedback.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Physical AI & Humanoid Robotics Guide"
      description="Learn to build intelligent robots from ROS 2 fundamentals to vision-language-action systems. Hands-on guide covering simulation, perception, and embodied AI.">
      <Hero />
      <main>
        <HomepageFeatures />
        <WhoThisIsFor />
        <WhatYouBuild />
      </main>
    </Layout>
  );
}
