import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A comprehensive AI-native textbook for embodied intelligence and robotics',
  favicon: 'img/robotics-logo.png', // Will need to add this image

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://physical-ai-book-uj5v.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For static hosting like Vercel, root is appropriate
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Bushraturk', // Usually your GitHub org/user name.
  projectName: 'my-website', // Usually your repo name.

  onBrokenLinks: 'ignore',

  // Markdown configuration to handle broken images
  markdown: {
    mermaid: true,
    mdx1Compat: {
      comments: true,
      admonitions: true,
      headingIds: true,
    },
  },

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
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Bushraturk/Physical-AI-Book/edit/main/',
          routeBasePath: '/', // Serve the docs at the root
        },
        theme: {
          customCss: './src/css/custom.css',
        },
        gtag: {
          trackingID: 'G-XXXXXXXXXX',
          anonymizeIP: true,
        },
        sitemap: {
          changefreq: 'weekly',
          priority: 0.5,
          ignorePatterns: ['/tags/**'],
          filename: 'sitemap.xml',
        },
        blog: false, // Disable blog functionality for textbook
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/robotics-social-card.jpg', // Will need to add this image
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    // Enable local search functionality
    algolia: undefined, // Explicitly disable Algolia to use local search
    navbar: {
      title: 'Physical AI & Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Textbook Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          type: 'search',
          position: 'right',
        },
        {
          href: 'https://github.com/Bushraturk/Physical-AI-Book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Modules',
          items: [
            {
              label: 'ROS 2',
              to: '/docs/ros2/intro',
            },
            {
              label: 'Gazebo/Unity',
              to: '/docs/gazebo-unity/intro',
            },
            {
              label: 'NVIDIA Isaac',
              to: '/docs/nvidia-isaac/intro',
            },
            {
              label: 'Vision-Language-Action',
              to: '/docs/vla/intro',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Hardware Setup',
              to: '/docs/hardware-guides',
            },
            {
              label: 'Lab Exercises',
              to: '/docs/ros2/lab-exercises/lab1',
            },
            {
              label: 'Assessments',
              to: '/docs/ros2/assessments/quiz1',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Instructor Guide',
              to: '/docs/instructor-guide/intro',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/Bushraturk/Physical-AI-Book',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Course. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'json', 'docker'],
    },
  } satisfies Preset.ThemeConfig,

  // Custom fields for non-standard configuration options
  customFields: {
    onBrokenMarkdownImages: 'warn', // Handle broken image references gracefully
  },
};

export default config;
