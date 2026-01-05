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
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'My First Chapter',
      items: [
        'my-first-chapter/lesson-1',
        'my-first-chapter/lesson-2',
        'my-first-chapter/lesson-3',
      ],
    },
    {
      type: 'category',
      label: 'Tutorial',
      items: [
        'tutorial-basics/create-a-document',
        'tutorial-basics/create-a-page',
        'tutorial-basics/deploy-your-site',
      ],
    },
    {
      type: 'category',
      label: 'Agentic AI + Robotics Course',
      items: [
        {
          type: 'category',
          label: 'Module 1: The Robotic Nervous System (ROS 2)',
          items: [
            'module-1-ros2/index',
            'module-1-ros2/what-is-ros2',
            'module-1-ros2/nodes-topics-services',
            'module-1-ros2/bridging-python-agents',
            'module-1-ros2/understanding-urdf',
            'module-1-ros2/real-world-relevance'
          ]
        },
        {
          type: 'category',
          label: 'Module 2: The Digital Twin (Gazebo & Unity)',
          items: [
            'module-2-digital-twin/index',
            'module-2-digital-twin/what-is-digital-twin',
            'module-2-digital-twin/gazebo-physics',
            'module-2-digital-twin/simulation-workflows',
            'module-2-digital-twin/unity-rendering',
            'module-2-digital-twin/sensor-simulation'
          ]
        },
        {
          type: 'category',
          label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
          items: [
            'module-3-isaac/index',
            'module-3-isaac/isaac-ecosystem',
            'module-3-isaac/isaac-sim',
            'module-3-isaac/isaac-ros',
            'module-3-isaac/nav2-navigation',
            'module-3-isaac/isaac-learning-outcomes'
          ]
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action (VLA)',
          items: [
            'module-4-vla/index',
            'module-4-vla/what-is-vla',
            'module-4-vla/voice-to-action',
            'module-4-vla/cognitive-planning',
            'module-4-vla/agentic-decision-making',
            'module-4-vla/vla-learning-outcomes'
          ]
        },
        {
          type: 'category',
          label: 'Capstone: The Autonomous Humanoid',
          items: [
            'capstone-autonomous-humanoid/index',
            'capstone-autonomous-humanoid/system-architecture',
            'capstone-autonomous-humanoid/implementation-guide',
            'capstone-autonomous-humanoid/skills-gained',
            'capstone-autonomous-humanoid/project-extensions',
            'capstone-autonomous-humanoid/evaluation-criteria',
            'capstone-autonomous-humanoid/module-integration'
          ]
        },
        'system-architecture',
        'review-and-polish',
        'course-summary'
      ]
    }
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
