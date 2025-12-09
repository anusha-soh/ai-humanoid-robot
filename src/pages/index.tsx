import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
// import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/intro">
            Start Reading →
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <div className="container" style={{padding: '2rem 0'}}>
          <div className="row">
            <div className="col col--4">
              <h3>🤖 ROS 2 Foundations</h3>
              <p>Master Robot Operating System 2, the industry standard for building robotic applications.</p>
            </div>
            <div className="col col--4">
              <h3>🎮 Simulation First</h3>
              <p>Learn with Gazebo and Isaac Sim - no hardware required to build autonomous robots.</p>
            </div>
            <div className="col col--4">
              <h3>🧠 Vision-Language-Action</h3>
              <p>Build autonomous humanoid robots with LLMs, computer vision, and advanced planning.</p>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
