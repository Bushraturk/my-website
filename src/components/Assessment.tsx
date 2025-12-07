import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './Assessment.module.css';

interface Question {
  id: string;
  question: string;
  type: 'multiple-choice' | 'true-false' | 'short-answer' | 'code';
  options?: string[]; // for multiple choice and true/false
  correctAnswer?: string | string[]; // for validation
}

interface AssessmentProps {
  title: string;
  description: string;
  questions: Question[];
  type: 'Quiz' | 'Assignment' | 'Project' | 'Practical';
  maxScore: number;
  timeLimit?: number; // in minutes
  onSubmit?: (score: number, answers: any[]) => void;
  showResults?: boolean;
}

const Assessment: React.FC<AssessmentProps> = ({
  title,
  description,
  questions,
  type,
  maxScore,
  timeLimit,
  onSubmit,
  showResults = false,
}) => {
  const [answers, setAnswers] = useState<any[]>(Array(questions.length).fill(null));
  const [submitted, setSubmitted] = useState(false);
  const [score, setScore] = useState<number | null>(null);
  const [timeLeft, setTimeLeft] = useState(timeLimit ? timeLimit * 60 : null); // in seconds

  // Timer effect if timeLimit is provided
  React.useEffect(() => {
    let interval: NodeJS.Timeout | null = null;
    
    if (timeLeft !== null && timeLeft > 0 && !submitted) {
      interval = setInterval(() => {
        setTimeLeft(prev => {
          if (prev === null || prev <= 1) {
            clearInterval(interval!);
            handleSubmit();
            return 0;
          }
          return prev - 1;
        });
      }, 1000);
    } else if (timeLeft === 0 && !submitted) {
      handleSubmit();
    }
    
    return () => {
      if (interval) clearInterval(interval);
    };
  }, [timeLeft, submitted]);

  const handleAnswerChange = (questionIndex: number, value: any) => {
    if (submitted) return;
    
    const newAnswers = [...answers];
    newAnswers[questionIndex] = value;
    setAnswers(newAnswers);
  };

  const handleSubmit = () => {
    if (submitted) return;
    
    // Calculate score (simplified for this example)
    let calculatedScore = 0;
    
    questions.forEach((question, index) => {
      if (question.correctAnswer && answers[index]) {
        if (Array.isArray(question.correctAnswer)) {
          // For multiple correct answers
          const userAnswerArray = Array.isArray(answers[index]) ? answers[index] : [answers[index]];
          const correctCount = question.correctAnswer.filter(ans => userAnswerArray.includes(ans)).length;
          const possiblePoints = question.correctAnswer.length;
          calculatedScore += (correctCount / possiblePoints) * (maxScore / questions.length);
        } else {
          // For single answer
          if (question.correctAnswer === answers[index]) {
            calculatedScore += maxScore / questions.length;
          }
        }
      }
    });
    
    setScore(Math.round(calculatedScore));
    setSubmitted(true);
    onSubmit?.(calculatedScore, answers);
  };

  const formatTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  };

  return (
    <div className={clsx('card', styles.assessment)}>
      <div className="card__header">
        <h2 className={styles.title}>{title}</h2>
        <div className={styles.metadata}>
          <div className={styles.type}>Type: {type}</div>
          <div className={styles.scoreInfo}>Max Score: {maxScore} points</div>
          {timeLimit && timeLeft !== null && (
            <div className={clsx(styles.timeRemaining, {
              [styles.timeWarning]: timeLeft < 300, // Less than 5 minutes
            })}>
              ⏱️ Time: {formatTime(timeLeft)}
            </div>
          )}
        </div>
      </div>
      
      <div className="card__body">
        <p className={styles.description}>{description}</p>
        
        {questions.map((question, qIndex) => (
          <div key={question.id} className={styles.question}>
            <h3 className={styles.questionTitle}>
              Question {qIndex + 1}: {question.question}
            </h3>
            
            {question.type === 'multiple-choice' && (
              <div className={styles.options}>
                {question.options?.map((option, oIndex) => (
                  <label key={oIndex} className={clsx(styles.option, styles.optionLabel)}>
                    <input
                      type="radio"
                      name={`question-${qIndex}`}
                      value={option}
                      checked={answers[qIndex] === option}
                      onChange={() => handleAnswerChange(qIndex, option)}
                      disabled={submitted}
                      className={styles.optionInput}
                    />
                    <span className={styles.optionText}>{option}</span>
                  </label>
                ))}
              </div>
            )}
            
            {question.type === 'true-false' && (
              <div className={styles.options}>
                {['True', 'False'].map((option, oIndex) => (
                  <label key={oIndex} className={clsx(styles.option, styles.optionLabel)}>
                    <input
                      type="radio"
                      name={`question-${qIndex}`}
                      value={option}
                      checked={answers[qIndex] === option}
                      onChange={() => handleAnswerChange(qIndex, option)}
                      disabled={submitted}
                      className={styles.optionInput}
                    />
                    <span className={styles.optionText}>{option}</span>
                  </label>
                ))}
              </div>
            )}
            
            {question.type === 'short-answer' && (
              <textarea
                value={answers[qIndex] || ''}
                onChange={(e) => handleAnswerChange(qIndex, e.target.value)}
                disabled={submitted}
                className={styles.textarea}
                rows={4}
              />
            )}
            
            {question.type === 'code' && (
              <div className={styles.codeContainer}>
                <textarea
                  value={answers[qIndex] || ''}
                  onChange={(e) => handleAnswerChange(qIndex, e.target.value)}
                  disabled={submitted}
                  className={clsx(styles.textarea, styles.codeInput)}
                  spellCheck={false}
                  placeholder="// Write your code here"
                />
              </div>
            )}
            
            {submitted && showResults && question.correctAnswer && (
              <div className={styles.feedback}>
                <div className={styles.answerStatus}>
                  {Array.isArray(question.correctAnswer)
                    ? answers[qIndex]?.toString() === question.correctAnswer.toString()
                      ? '✅ Correct!'
                      : `❌ Incorrect. Correct answer: ${question.correctAnswer.join(', ')}`
                    : answers[qIndex] === question.correctAnswer
                    ? '✅ Correct!'
                    : `❌ Incorrect. Correct answer: ${question.correctAnswer}`}
                </div>
              </div>
            )}
          </div>
        ))}
        
        {!submitted ? (
          <div className={styles.submitSection}>
            <button 
              onClick={handleSubmit}
              disabled={answers.some(a => a === null)}
              className={clsx('button button--primary', styles.submitButton)}
            >
              Submit Assessment
            </button>
            <p className={styles.disclaimer}>
              Note: Once submitted, you cannot change your answers.
            </p>
          </div>
        ) : (
          <div className={clsx('alert', 'alert--success', styles.results)}>
            <h3>Assessment Results</h3>
            <p>
              You scored <strong>{score}</strong> out of <strong>{maxScore}</strong> points.
              {score !== null && (
                <span> ({Math.round((score / maxScore) * 100)}%)</span>
              )}
            </p>
          </div>
        )}
      </div>
    </div>
  );
};

export default Assessment;