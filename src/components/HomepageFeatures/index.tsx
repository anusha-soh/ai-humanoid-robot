import React from 'react';
import styles from './styles.module.css';

// Inline SVG imports for optimal performance
import NervousSystemIcon from '@site/static/img/icons/module-01-nervous-system.svg';
import DigitalTwinIcon from '@site/static/img/icons/module-02-digital-twin.svg';
import AIBrainIcon from '@site/static/img/icons/module-03-ai-brain.svg';
import VLAIcon from '@site/static/img/icons/module-04-vla.svg';

interface ModuleCardProps {
  Icon: React.ComponentType<React.SVGProps<SVGSVGElement>>;
  title: string;
  description: string;
  ariaLabel: string;
}

const ModuleCard: React.FC<ModuleCardProps> = ({ Icon, title, description, ariaLabel }) => {
  return (
    <div className={styles.moduleCard}>
      <div className={styles.moduleIcon}>
        <Icon role="img" aria-label={ariaLabel} />
      </div>
      <h3 className={styles.moduleTitle}>{title}</h3>
      <p className={styles.moduleDescription}>{description}</p>
    </div>
  );
};

const modules: ModuleCardProps[] = [
  {
    Icon: NervousSystemIcon,
    title: 'Module 1: Robotic Nervous System',
    description: 'Master ROS 2 fundamentals—nodes, topics, services, and actions. Build distributed robotic systems with pub/sub messaging and low-level control.',
    ariaLabel: 'ROS 2 Robotic Nervous System - Interconnected nodes representing communication',
  },
  {
    Icon: DigitalTwinIcon,
    title: 'Module 2: Digital Twin',
    description: 'Create realistic robot simulations in Gazebo. Design URDF models, tune physics parameters, and test algorithms in virtual environments before hardware deployment.',
    ariaLabel: 'Gazebo Digital Twin - 3D cube representing simulation environment',
  },
  {
    Icon: AIBrainIcon,
    title: 'Module 3: AI-Robot Brain',
    description: 'Integrate perception and planning with NVIDIA Isaac Sim. Implement computer vision, sensor fusion, and reinforcement learning for autonomous robot behavior.',
    ariaLabel: 'Isaac Sim AI Brain - Camera icon representing vision and perception systems',
  },
  {
    Icon: VLAIcon,
    title: 'Module 4: Vision-Language-Action',
    description: 'Build embodied AI agents using vision-language models. Enable robots to understand natural language commands and translate them into physical actions.',
    ariaLabel: 'Vision-Language-Action - Brain with communication representing multimodal AI',
  },
];

export default function HomepageFeatures(): JSX.Element {
  return (
    <section className={styles.features}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>What You'll Learn</h2>
        <div className={styles.moduleGrid}>
          {modules.map((props, idx) => (
            <ModuleCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
