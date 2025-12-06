import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for Physical AI & Humanoid Robotics Book
 *
 * Structure: 4 modules × 4 chapters each = 16 chapters total
 * - Module 1: Robotic Nervous System (ROS 2 Foundations)
 * - Module 2: Digital Twin (Gazebo & Simulation)
 * - Module 3: AI-Robot Brain (NVIDIA Isaac & Perception)
 * - Module 4: Vision-Language-Action (VLA Systems)
 */
const sidebars: SidebarsConfig = {
  bookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: Robotic Nervous System',
      collapsed: false,
      items: [
        'module-01/chapter-01',
        'module-01/chapter-02',
        'module-01/chapter-03',
        'module-01/chapter-04',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      collapsed: true,
      items: [
        'module-02/chapter-05',
        'module-02/chapter-06',
        'module-02/chapter-07',
        'module-02/chapter-08',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain',
      collapsed: true,
      items: [
        'module-03/chapter-09',
        'module-03/chapter-10',
        'module-03/chapter-11',
        'module-03/chapter-12',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      collapsed: true,
      items: [
        'module-04/chapter-13',
        'module-04/chapter-14',
        'module-04/chapter-15',
        'module-04/chapter-16',
      ],
    },
    {
      type: 'category',
      label: 'Appendix',
      collapsed: true,
      items: [
        'appendix/prerequisites',
        'appendix/glossary',
        'appendix/troubleshooting',
        'appendix/resources',
      ],
    },
  ],
};

export default sidebars;
