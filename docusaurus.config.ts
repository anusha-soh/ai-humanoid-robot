import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Practical Guide to ROS 2, Simulation, and Vision-Language-Action Systems',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://ai-humanoid-robot.vercel.app', // Replace with your actual Vercel URL
  // Set the /<baseUrl>/ pathname under which your site is served
  // For Vercel deployment, use root path
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'anusha-soh', // Usually your GitHub org/user name.
  projectName: 'ai-humanoid-robot', // Usually your repo name.
  trailingSlash: false,

  onBrokenLinks: 'warn', // Temporarily warn instead of throw during setup

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: '/', // Serve docs at the site's root
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/anusha-soh/ai-humanoid-robot/tree/main/',
        },
        blog: false, // Disable blog for book project
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Book Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/anusha-soh/ai-humanoid-robot',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book',
          items: [
            {
              label: 'Introduction',
              to: '/intro',
            },
            {
              label: 'Getting Started',
              to: '/intro',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Prerequisites',
              to: '/appendix/prerequisites',
            },
            {
              label: 'Glossary',
              to: '/appendix/glossary',
            },
            {
              label: 'Troubleshooting',
              to: '/appendix/troubleshooting',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/anusha-soh/ai-humanoid-robot',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml'], // Add syntax highlighting for Python, Bash, YAML
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
