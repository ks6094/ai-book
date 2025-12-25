import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';


const FeatureList = [
  {
    title: '🤖 The Robotic Nervous System',
    Svg: null,
    description: (
      <>
        Learn ROS 2 fundamentals for robot communication and control
      </>
    ),
  },
  {
    title: '🎮 Digital Twin Simulation',
    Svg: null,
    description: (
      <>
        Master physics-accurate simulation with Gazebo and Unity
      </>
    ),
  },
  {
    title: '🧠 AI-Robot Brain',
    Svg: null,
    description: (
      <>
        Implement intelligent perception and navigation with NVIDIA Isaac
      </>
    ),
  },
  {
    title: '💬 Vision-Language-Action',
    Svg: null,
    description: (
      <>
        Create robots that respond to natural language commands
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}