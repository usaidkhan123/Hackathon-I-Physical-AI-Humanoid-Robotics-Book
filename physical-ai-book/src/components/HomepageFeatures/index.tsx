import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type ModuleItem = {
  title: string;
  link: string;
  description: React.ReactNode;
};

const ModuleList: ModuleItem[] = [
  {
    title: 'Module 1: The Foundations of ROS 2',
    link: '/docs/module-01-ros2/module-01-ros2',
    description: (
      <>
        Start your journey by mastering the core concepts of the Robot Operating System 2 (ROS 2), the backbone of modern robotics.
      </>
    ),
  },
  {
    title: 'Module 2: Building Virtual Worlds',
    link: '/docs/module-02-simulation/module-02-simulation',
    description: (
      <>
        Learn to simulate robots in realistic environments using Gazebo and Unity, essential skills for testing and development.
      </>
    ),
  },
  {
    title: 'Module 3: NVIDIA Isaac and AI',
    link: '/docs/module-03-nvidia-isaac/module-03-nvidia-isaac',
    description: (
      <>
        Dive into the powerful NVIDIA Isaac SDK for accelerated AI robotics, including perception, navigation, and manipulation.
      </>
    ),
  },
];

function Module({title, description, link}: ModuleItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className={clsx('card', styles.moduleCard)}>
        <div className="card__header">
          <Heading as="h3">{title}</Heading>
        </div>
        <div className="card__body">
          <p>{description}</p>
        </div>
        <div className="card__footer">
          <a href={link} className="button button--primary button--block">
            Explore Module
          </a>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): React.ReactElement {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--12 text--center">
            <Heading as="h2" className={styles.sectionTitle}>Explore the Curriculum</Heading>
          </div>
        </div>
        <div className="row">
          {ModuleList.map((props, idx) => (
            <Module key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}