import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './LabExercise.module.css';

interface LabExerciseProps {
  title: string;
  description: string;
  objectives: string[];
  prerequisites: string[];
  steps: string[];
  difficulty: 'Beginner' | 'Intermediate' | 'Advanced';
  estimatedDuration: number; // in minutes
  requiredEquipment?: string[];
  onCompletion?: () => void;
}

const LabExercise: React.FC<LabExerciseProps> = ({
  title,
  description,
  objectives,
  prerequisites,
  steps,
  difficulty,
  estimatedDuration,
  requiredEquipment = [],
  onCompletion,
}) => {
  const [completedSteps, setCompletedSteps] = useState<boolean[]>(
    Array(steps.length).fill(false)
  );
  const [isCompleted, setIsCompleted] = useState(false);

  const toggleStep = (index: number) => {
    const newCompletedSteps = [...completedSteps];
    newCompletedSteps[index] = !newCompletedSteps[index];
    setCompletedSteps(newCompletedSteps);

    // Check if all steps are completed
    const allStepsCompleted = newCompletedSteps.every(step => step);
    if (allStepsCompleted) {
      setIsCompleted(true);
      onCompletion?.();
    }
  };

  // Calculate completion percentage
  const completionPercentage = Math.round(
    (completedSteps.filter(Boolean).length / steps.length) * 100
  );

  const difficultyColors = {
    Beginner: 'var(--ifm-color-success)',
    Intermediate: 'var(--ifm-color-warning)',
    Advanced: 'var(--ifm-color-danger)',
  };

  const difficultyColor = difficultyColors[difficulty] || 'var(--ifm-color-primary)';

  return (
    <div className={clsx('card', styles.labExercise)}>
      <div className="card__header">
        <h2 className={styles.title}>{title}</h2>
        <div className={styles.metadata}>
          <div className={styles.difficulty} style={{ color: difficultyColor }}>
            {difficulty}
          </div>
          <div className={styles.duration}>
            ⏱️ ~{estimatedDuration} minutes
          </div>
        </div>
      </div>
      
      <div className="card__body">
        <div className={styles.progressBar}>
          <div 
            className={styles.progressFill} 
            style={{ width: `${completionPercentage}%` }}
          />
          <span className={styles.progressText}>
            {completionPercentage}% Complete
          </span>
        </div>

        <p className={styles.description}>{description}</p>

        <div className={styles.section}>
          <h3>Learning Objectives</h3>
          <ul>
            {objectives.map((objective, idx) => (
              <li key={idx}>{objective}</li>
            ))}
          </ul>
        </div>

        {prerequisites.length > 0 && (
          <div className={styles.section}>
            <h3>Prerequisites</h3>
            <ul>
              {prerequisites.map((prereq, idx) => (
                <li key={idx}>{prereq}</li>
              ))}
            </ul>
          </div>
        )}

        {requiredEquipment.length > 0 && (
          <div className={styles.section}>
            <h3>Required Equipment</h3>
            <ul>
              {requiredEquipment.map((equipment, idx) => (
                <li key={idx}>{equipment}</li>
              ))}
            </ul>
          </div>
        )}

        <div className={styles.section}>
          <h3>Lab Steps</h3>
          <ol>
            {steps.map((step, idx) => (
              <li key={idx} className={styles.stepItem}>
                <label className={styles.stepLabel}>
                  <input
                    type="checkbox"
                    checked={completedSteps[idx]}
                    onChange={() => toggleStep(idx)}
                    className={styles.stepCheckbox}
                  />
                  <span className={clsx(styles.stepText, {
                    [styles.stepCompleted]: completedSteps[idx]
                  })}>
                    {step}
                  </span>
                </label>
              </li>
            ))}
          </ol>
        </div>
      </div>

      {isCompleted && (
        <div className={clsx('alert', 'alert--success', styles.completionAlert)}>
          🎉 Congratulations! You have completed this lab exercise.
        </div>
      )}
    </div>
  );
};

export default LabExercise;