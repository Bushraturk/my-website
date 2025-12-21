import React from 'react';
import Link from '@docusaurus/Link';
import Translate from '@docusaurus/Translate';

function Footer(): JSX.Element | null {
  const currentYear = new Date().getFullYear();

  return (
    <footer className="footer footer--dark">
      <div className="container container-fluid">
        <div className="row footer__links">
          <div className="col footer__col">
            <div className="footer__title">
              <Translate id="footer.courseModules.title">Course Modules</Translate>
            </div>
            <ul className="footer__items clean-list">
              <li className="footer__item">
                <Link className="footer__link-item" to="/ros2/intro">
                  <Translate id="footer.courseModules.ros2">ROS 2</Translate>
                </Link>
              </li>
              <li className="footer__item">
                <Link className="footer__link-item" to="/gazebo-unity/intro">
                  <Translate id="footer.courseModules.gazebo">Gazebo/Unity</Translate>
                </Link>
              </li>
              <li className="footer__item">
                <Link className="footer__link-item" to="/nvidia-isaac/intro">
                  <Translate id="footer.courseModules.isaac">NVIDIA Isaac</Translate>
                </Link>
              </li>
              <li className="footer__item">
                <Link className="footer__link-item" to="/vla/intro">
                  <Translate id="footer.courseModules.vla">VLA</Translate>
                </Link>
              </li>
            </ul>
          </div>
          <div className="col footer__col">
            <div className="footer__title">
              <Translate id="footer.resources.title">Resources</Translate>
            </div>
            <ul className="footer__items clean-list">
              <li className="footer__item">
                <Link className="footer__link-item" to="/ros2/lab-exercises/lab1">
                  <Translate id="footer.resources.labs">Lab Exercises</Translate>
                </Link>
              </li>
              <li className="footer__item">
                <Link className="footer__link-item" to="/ros2/assessments/quiz1">
                  <Translate id="footer.resources.assessments">Assessments</Translate>
                </Link>
              </li>
              <li className="footer__item">
                <Link className="footer__link-item" to="/instructor-guide/intro">
                  <Translate id="footer.resources.instructor">Instructor Guide</Translate>
                </Link>
              </li>
            </ul>
          </div>
          <div className="col footer__col">
            <div className="footer__title">
              <Translate id="footer.community.title">Community</Translate>
            </div>
            <ul className="footer__items clean-list">
              <li className="footer__item">
                <a
                  className="footer__link-item"
                  href="https://github.com/Bushraturk/my-website"
                  target="_blank"
                  rel="noopener noreferrer"
                >
                  GitHub
                </a>
              </li>
            </ul>
          </div>
        </div>
        <div className="footer__bottom text--center">
          <div className="footer__copyright">
            <Translate
              id="footer.copyright"
              values={{ year: currentYear.toString() }}
            >
              {'Copyright © {year} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.'}
            </Translate>
          </div>
        </div>
      </div>
    </footer>
  );
}

export default React.memo(Footer);
