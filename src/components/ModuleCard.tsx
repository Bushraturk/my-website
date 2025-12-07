import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './ModuleCard.module.css';

interface ModuleCardProps {
  title: string;
  description: string;
  weeks: string;
  difficulty: 'Beginner' | 'Intermediate' | 'Advanced';
  estimatedHours: number;
  to: string;
  icon?: string;
}

const difficultyColors = {
  Beginner: 'var(--ifm-color-success)',
  Intermediate: 'var(--ifm-color-warning)',
  Advanced: 'var(--ifm-color-danger)',
};

const ModuleCard: React.FC<ModuleCardProps> = ({
  title,
  description,
  weeks,
  difficulty,
  estimatedHours,
  to,
  icon,
}) => {
  const difficultyColor = difficultyColors[difficulty] || 'var(--ifm-color-primary)';

  return (
    <div className={clsx('card', styles.moduleCard)}>
      <div className={clsx('card__header', styles.cardHeader)}>
        {icon && <div className={styles.icon}>{icon}</div>}
        <div className={styles.headerText}>
          <h3 className={clsx('text--truncate', styles.title)}>
            <Link href={to} className={styles.titleLink}>
              {title}
            </Link>
          </h3>
          <div className={styles.weeks}>
            <span className={styles.weekLabel}>Weeks:</span> {weeks}
          </div>
        </div>
      </div>
      <div className="card__body">
        <p className={styles.description}>{description}</p>
        <div className={styles.metadata}>
          <div className={styles.difficulty}>
            <span className={styles.label}>Difficulty:</span>
            <span 
              className={styles.difficultyValue}
              style={{ color: difficultyColor }}
            >
              {difficulty}
            </span>
          </div>
          <div className={styles.time}>
            <span className={styles.label}>Time:</span>
            <span className={styles.timeValue}>
              ~{estimatedHours} hours
            </span>
          </div>
        </div>
      </div>
      <div className="card__footer">
        <Link className="button button--primary" href={to}>
          Start Module
        </Link>
      </div>
    </div>
  );
};

export default ModuleCard;