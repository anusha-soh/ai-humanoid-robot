import React from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

export default function Hero(): JSX.Element {
  return (
    <header className={styles.hero}>
      <div className={styles.heroContent}>
        <h1 className={styles.heroTitle}>
          Physical AI & Humanoid Robotics
        </h1>
        <p className={styles.heroTagline}>
          A Practical Guide to ROS 2, Simulation, and Vision-Language-Action Systems
        </p>
        <p className={styles.heroDescription}>
          Learn to build intelligent robots from the ground up—from low-level control with ROS 2 to high-level reasoning with vision-language models. This hands-on guide covers the complete stack for creating embodied AI systems.
        </p>
        <div className={styles.heroButtons}>
          <Link
            className={styles.heroCta}
            to="/intro"
          >
            Get Started
          </Link>
        </div>
      </div>
    </header>
  );
}
