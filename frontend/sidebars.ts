import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

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
      label: 'Module 1: Robot Operating System (ROS 2) - Foundation & Communication',
      items: [
        {
          type: 'category',
          label: 'Week 1: Introduction to ROS 2',
          items: [
            'module-1/week-1-introduction/1-1-what-is-ros2',
            'module-1/week-1-introduction/1-2-architecture',
            'module-1/week-1-introduction/1-3-installation-setup',
            'module-1/week-1-introduction/1-4-practical-exercises',
          ],
        },
        {
          type: 'category',
          label: 'Week 2: Nodes and Topics',
          items: [
            'module-1/week-2-nodes-topics/2-1-understanding-nodes',
            'module-1/week-2-nodes-topics/2-2-working-with-topics',
            'module-1/week-2-nodes-topics/2-3-topic-design-patterns',
            'module-1/week-2-nodes-topics/2-4-practical-topics-exercises',
          ],
        },
        {
          type: 'category',
          label: 'Week 3: Services and Actions',
          items: [
            'module-1/week-3-services-actions/3-1-understanding-services',
            'module-1/week-3-services-actions/3-2-understanding-actions',
            'module-1/week-3-services-actions/3-3-comparison-topic-service-action',
            'module-1/week-3-services-actions/3-4-communication-practical-exercises',
          ],
        },
        {
          type: 'category',
          label: 'Week 4: TF2 and URDF',
          items: [
            'module-1/week-4-tf-urdf/4-1-understanding-tf2',
            'module-1/week-4-tf-urdf/4-2-understanding-urdf',
            'module-1/week-4-tf-urdf/4-3-integration-tf2-urdf',
            'module-1/week-4-tf-urdf/4-4-tf2-urdf-practical-exercises',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Robot Simulation - Virtual Testing Environments',
      items: [
        {
          type: 'category',
          label: 'Week 1: Introduction to Simulation',
          items: [
            'module-2/week-1-introduction/2-1-overview-of-robot-simulation',
            'module-2/week-1-introduction/2-2-gazebo-installation-setup',
            'module-2/week-1-introduction/2-3-basic-simulation-concepts',
            'module-2/week-1-introduction/2-4-simulation-practical-exercises',
          ],
        },
        {
          type: 'category',
          label: 'Week 2: Gazebo Basics',
          items: [
            'module-2/week-2-gazebo-basics/2-1-introduction-to-gazebo',
          ],
        },
        {
          type: 'category',
          label: 'Week 3: Simulation Integration',
          items: [
            'module-2/week-3-simulation-integration/3-1-simulation-integration-techniques',
          ],
        },
        {
          type: 'category',
          label: 'Week 4: Advanced Simulation',
          items: [
            'module-2/week-4-advanced-simulation/4-1-advanced-simulation-techniques',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Sim - Advanced Photorealistic Simulation',
      items: [
        {
          type: 'category',
          label: 'Week 1: Introduction to Isaac Sim',
          items: [
            'module-3/week-1-introduction/3-1-overview-of-nvidia-isaac-sim',
            'module-3/week-1-introduction/3-2-isaac-sim-installation-setup',
            'module-3/week-1-introduction/3-3-isaac-sim-basics-robotics',
            'module-3/week-1-introduction/3-4-isaac-sim-practical-exercises',
          ],
        },
        {
          type: 'category',
          label: 'Week 2: Isaac ROS Basics',
          items: [
            'module-3/week-2-isaac-ros-basics/2-1-introduction-to-isaac-ros',
          ],
        },
        {
          type: 'category',
          label: 'Week 3: Advanced Isaac Sim',
          items: [
            'module-3/week-3-advanced-isaac-sim/3-1-advanced-isaac-sim-techniques',
          ],
        },
        {
          type: 'category',
          label: 'Week 4: Isaac Sim Applications',
          items: [
            'module-3/week-4-isaac-sim-applications/4-1-isaac-sim-applications',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) Models - Multimodal AI for Robotics',
      items: [
        {
          type: 'category',
          label: 'Week 1: Introduction to VLA Models',
          items: [
            'module-4/week-1-introduction/4-1-overview-of-vla-models',
            'module-4/week-1-introduction/4-2-vla-architecture-deep-learning',
            'module-4/week-1-introduction/4-3-vla-training-data-collection',
            'module-4/week-1-introduction/4-4-vla-practical-implementation',
          ],
        },
        {
          type: 'category',
          label: 'Week 2: VLA Fundamentals',
          items: [
            'module-4/week-2-vla-fundamentals/2-1-introduction-to-vla-models',
          ],
        },
        {
          type: 'category',
          label: 'Week 3: VLA Integration',
          items: [
            'module-4/week-3-vla-integration/3-1-vla-integration-with-robotics',
          ],
        },
        {
          type: 'category',
          label: 'Week 4: Advanced VLA Applications',
          items: [
            'module-4/week-4-advanced-vla-applications/4-1-advanced-vla-applications',
          ],
        },
      ],
    },
  ],
};

export default sidebars;