import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  textbookSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: [
        'intro',
        'home-test',
        'translation-validation-test',
        'translation-process-guide',
        'mdx-components-guide',
        'mdx-troubleshooting-guide',
        'translation-feature-migration',
      ],
      link: {
        type: 'doc',
        id: 'intro',
      },
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 - Robotic Nervous System',
      collapsible: true,
      collapsed: false,
      items: [
        'ros2/intro',
        'ros2/week1-2',
        'ros2/week3',
        {
          type: 'category',
          label: 'ROS 2 Lab Exercises',
          collapsed: true,
          items: [
            'ros2/lab-exercises/lab1',
            'ros2/lab-exercises/lab2'
          ]
        },
        {
          type: 'category',
          label: 'ROS 2 Assessments',
          collapsed: true,
          items: [
            'ros2/assessments/quiz1',
            'ros2/assessments/assignment1'
          ]
        },
        'ros2/conclusion',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo/Unity - Digital Twin',
      collapsible: true,
      collapsed: false,
      items: [
        'gazebo-unity/intro',
        'gazebo-unity/week4-5',
        'gazebo-unity/week6',
        {
          type: 'category',
          label: 'Gazebo/Unity Lab Exercises',
          collapsed: true,
          items: [
            'gazebo-unity/lab-exercises/lab1',
            'gazebo-unity/lab-exercises/lab2'
          ]
        },
        {
          type: 'category',
          label: 'Gazebo/Unity Assessments',
          collapsed: true,
          items: [
            'gazebo-unity/assessments/quiz1',
            'gazebo-unity/assessments/assignment1'
          ]
        },
        'gazebo-unity/conclusion',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac - AI Robot Brains',
      collapsible: true,
      collapsed: false,
      items: [
        'nvidia-isaac/intro',
        'nvidia-isaac/week7-8',
        'nvidia-isaac/week9',
        {
          type: 'category',
          label: 'NVIDIA Isaac Lab Exercises',
          collapsed: true,
          items: [
            'nvidia-isaac/lab-exercises/lab1'
          ]
        },
        {
          type: 'category',
          label: 'NVIDIA Isaac Assessments',
          collapsed: true,
          items: [
            'nvidia-isaac/assessments/quiz1',
            'nvidia-isaac/assessments/assignment1'
          ]
        },
        'nvidia-isaac/conclusion',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA - Vision-Language-Action',
      collapsible: true,
      collapsed: false,
      items: [
        'vla/intro',
        'vla/week10-11',
        'vla/week12',
        'vla/week13',
        {
          type: 'category',
          label: 'VLA Lab Exercises',
          collapsed: true,
          items: [
            'vla/lab-exercises/lab1'
          ]
        },
        {
          type: 'category',
          label: 'VLA Assessments',
          collapsed: true,
          items: [
            'vla/assessments/quiz1',
            'vla/assessments/assignment1'
          ]
        },
        'vla/conclusion',
      ],
    },
    {
      type: 'doc',
      id: 'conclusion',
    },
    {
      type: 'doc',
      id: 'home-test',
    },
    {
      type: 'doc',
      id: 'test-mdx-component',
    },
    {
      type: 'category',
      label: 'Instructor Guide',
      collapsible: true,
      collapsed: true,
      items: [
        'instructor-guide/intro',
        'instructor-guide/offline-access',
      ],
    },
    {
      type: 'category',
      label: 'Documentation',
      collapsible: true,
      collapsed: true,
      items: [
        'translation-validation-test',
        'translation-process-guide',
        'translation-feature-migration',
        'mdx-components-guide',
        'mdx-troubleshooting-guide',
      ],
    },
  ],
};

export default sidebars;
