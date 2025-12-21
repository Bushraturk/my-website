import React, { useState } from 'react';
import { useAuth } from '../contexts/AuthContext';
import Translate from '@docusaurus/Translate';

// Accessible Module Card Component with proper ARIA attributes
const AccessibleModuleCard = ({ 
  title, 
  description, 
  difficulty, 
  weeks, 
  progress, 
  link,
  children 
}) => {
  const [isExpanded, setIsExpanded] = useState(false);
  const { user } = useAuth();
  
  // Toggle expanded state for screen reader users
  const toggleExpanded = () => {
    setIsExpanded(!isExpanded);
  };

  return (
    <div 
      className="module-card"
      role="region"
      aria-labelledby={`module-title-${title.replace(/\s+/g, '-').toLowerCase()}`}
      tabIndex={-1}
    >
      <div 
        className="module-card-header"
        onClick={toggleExpanded}
        onKeyDown={(e) => {
          if (e.key === 'Enter' || e.key === ' ') {
            e.preventDefault();
            toggleExpanded();
          }
        }}
        role="button"
        tabIndex={0}
        aria-expanded={isExpanded}
        aria-controls={`module-content-${title.replace(/\s+/g, '-').toLowerCase()}`}
      >
        <h3 
          id={`module-title-${title.replace(/\s+/g, '-').toLowerCase()}`}
          className="module-card-title"
        >
          {title}
        </h3>
        <div className="module-card-meta">
          <span className="module-card-difficulty" aria-label={`Difficulty: ${difficulty}`}>
            <Translate id="module.difficulty" defaultMessage="Difficulty" />: {difficulty}
          </span>
          <span className="module-card-weeks" aria-label={`Duration: ${weeks}`}>
            <Translate id="module.duration" defaultMessage="Duration" />: {weeks}
          </span>
          {user && (
            <span className="module-card-progress" aria-label={`Progress: ${progress}`}>
              <Translate id="module.progress" defaultMessage="Progress" />: {progress}%
            </span>
          )}
        </div>
      </div>
      
      <div 
        id={`module-content-${title.replace(/\s+/g, '-').toLowerCase()}`}
        className={`module-card-content ${isExpanded ? 'expanded' : 'collapsed'}`}
        aria-hidden={!isExpanded}
        style={{ display: isExpanded ? 'block' : 'none' }}
      >
        <p className="module-card-description">{description}</p>
        {children}
        {link && (
          <a 
            href={link} 
            className="module-card-link"
            aria-label={`Go to ${title} module`}
          >
            <Translate id="module.start-learning" defaultMessage="Start Learning" />
          </a>
        )}
      </div>
    </div>
  );
};

// Accessible Lab Exercise Component
const AccessibleLabExercise = ({ 
  title, 
  objectives, 
  duration, 
  difficulty, 
  prerequisites,
  content 
}) => {
  const [showPrerequisites, setShowPrerequisites] = useState(false);
  
  return (
    <div 
      className="lab-exercise-container"
      role="article"
      aria-labelledby={`lab-title-${title.replace(/\s+/g, '-').toLowerCase()}`}
    >
      <header className="lab-header">
        <h3 
          id={`lab-title-${title.replace(/\s+/g, '-').toLowerCase()}`}
          className="lab-title"
        >
          {title}
        </h3>
        <div className="lab-meta" aria-label="Lab exercise metadata">
          <span className="lab-duration" aria-label={`Duration: ${duration}`}>
            <Translate id="lab.duration" defaultMessage="Duration" />: {duration}
          </span>
          <span className="lab-difficulty" aria-label={`Difficulty: ${difficulty}`}>
            <Translate id="lab.difficulty" defaultMessage="Difficulty" />: {difficulty}
          </span>
        </div>
      </header>
      
      <section 
        className="lab-objectives"
        aria-labelledby={`objectives-title-${title.replace(/\s+/g, '-').toLowerCase()}`}
      >
        <h4 
          id={`objectives-title-${title.replace(/\s+/g, '-').toLowerCase()}`}
          className="lab-section-title"
        >
          <Translate id="lab.objectives" defaultMessage="Learning Objectives" />
        </h4>
        <ul>
          {objectives.map((objective, index) => (
            <li key={index} aria-label={objective}>
              {objective}
            </li>
          ))}
        </ul>
      </section>
      
      <section 
        className="lab-prerequisites"
        aria-labelledby={`prerequisites-title-${title.replace(/\s+/g, '-').toLowerCase()}`}
      >
        <button
          onClick={() => setShowPrerequisites(!showPrerequisites)}
          aria-expanded={showPrerequisites}
          aria-controls={`prereq-content-${title.replace(/\s+/g, '-').toLowerCase()}`}
          className="lab-prereq-toggle"
        >
          <Translate id="lab.prerequisites" defaultMessage="Prerequisites" />
        </button>
        <div 
          id={`prereq-content-${title.replace(/\s+/g, '-').toLowerCase()}`}
          className={`lab-prereq-content ${showPrerequisites ? 'visible' : 'hidden'}`}
          aria-hidden={!showPrerequisites}
        >
          <ul>
            {prerequisites.map((prereq, index) => (
              <li key={index}>{prereq}</li>
            ))}
          </ul>
        </div>
      </section>
      
      <section 
        className="lab-content"
        aria-labelledby={`content-title-${title.replace(/\s+/g, '-').toLowerCase()}`}
      >
        <h4 
          id={`content-title-${title.replace(/\s+/g, '-').toLowerCase()}`}
          className="lab-section-title"
        >
          <Translate id="lab.content" defaultMessage="Lab Content" />
        </h4>
        <div className="lab-steps">
          {content}
        </div>
      </section>
    </div>
  );
};

// Accessible Assessment Component
const AccessibleAssessment = ({ 
  title, 
  type, 
  description, 
  questions,
  onSubmit 
}) => {
  const [currentQuestionIndex, setCurrentQuestionIndex] = useState(0);
  const [answers, setAnswers] = useState({});
  const [isSubmitted, setIsSubmitted] = useState(false);
  
  const handleAnswerChange = (questionId, value) => {
    setAnswers(prev => ({
      ...prev,
      [questionId]: value
    }));
  };
  
  const handleNextQuestion = () => {
    if (currentQuestionIndex < questions.length - 1) {
      setCurrentQuestionIndex(currentQuestionIndex + 1);
    }
  };
  
  const handlePreviousQuestion = () => {
    if (currentQuestionIndex > 0) {
      setCurrentQuestionIndex(currentQuestionIndex - 1);
    }
  };
  
  const handleSubmit = () => {
    setIsSubmitted(true);
    if (onSubmit) {
      onSubmit(answers);
    }
  };
  
  if (isSubmitted) {
    return (
      <div 
        className="assessment-result"
        role="status"
        aria-live="polite"
      >
        <h3><Translate id="assessment.submitted" defaultMessage="Assessment Submitted" /></h3>
        <p><Translate 
          id="assessment.thanks" 
          defaultMessage="Thank you for completing the assessment." 
        /></p>
      </div>
    );
  }
  
  const currentQuestion = questions[currentQuestionIndex];
  
  return (
    <div 
      className="assessment-container"
      role="form"
      aria-labelledby={`assessment-title-${title.replace(/\s+/g, '-').toLowerCase()}`}
    >
      <header className="assessment-header">
        <h3 
          id={`assessment-title-${title.replace(/\s+/g, '-').toLowerCase()}`}
          className="assessment-title"
        >
          {title} (<Translate id={`assessment.${type}`} defaultMessage={type} />)
        </h3>
        <p className="assessment-description">{description}</p>
      </header>
      
      <div 
        className="assessment-progress"
        role="progressbar"
        aria-valuenow={currentQuestionIndex + 1}
        aria-valuemin={1}
        aria-valuemax={questions.length}
        aria-label={`Question ${currentQuestionIndex + 1} of ${questions.length}`}
      >
        <span>
          <Translate 
            id="assessment.progress" 
            defaultMessage="Question {current} of {total}" 
            values={{
              current: currentQuestionIndex + 1,
              total: questions.length
            }}
          />
        </span>
      </div>
      
      <div className="assessment-question">
        <h4 
          id={`question-${currentQuestion.id}`}
          className="question-text"
        >
          {currentQuestion.text}
        </h4>
        
        <div className="question-choices">
          {currentQuestion.choices?.map((choice, index) => (
            <div key={index} className="choice-option">
              <input
                type={currentQuestion.type === 'multiple-choice' ? 'radio' : 'checkbox'}
                id={`choice-${currentQuestion.id}-${index}`}
                name={`question-${currentQuestion.id}`}
                value={choice.value}
                checked={answers[currentQuestion.id] === choice.value}
                onChange={(e) => handleAnswerChange(currentQuestion.id, e.target.value)}
                aria-describedby={`question-${currentQuestion.id}`}
              />
              <label 
                htmlFor={`choice-${currentQuestion.id}-${index}`}
                className="choice-label"
              >
                {choice.text}
              </label>
            </div>
          ))}
        </div>
        
        {currentQuestion.type === 'open-ended' && (
          <textarea
            id={`answer-${currentQuestion.id}`}
            value={answers[currentQuestion.id] || ''}
            onChange={(e) => handleAnswerChange(currentQuestion.id, e.target.value)}
            aria-describedby={`question-${currentQuestion.id}`}
            rows={4}
            className="open-ended-input"
          />
        )}
      </div>
      
      <div className="assessment-navigation">
        <button
          onClick={handlePreviousQuestion}
          disabled={currentQuestionIndex === 0}
          aria-label="Previous question"
        >
          <Translate id="assessment.previous" defaultMessage="Previous" />
        </button>
        
        {currentQuestionIndex < questions.length - 1 ? (
          <button
            onClick={handleNextQuestion}
            aria-label="Next question"
          >
            <Translate id="assessment.next" defaultMessage="Next" />
          </button>
        ) : (
          <button
            onClick={handleSubmit}
            aria-label="Submit assessment"
          >
            <Translate id="assessment.submit" defaultMessage="Submit" />
          </button>
        )}
      </div>
    </div>
  );
};

export { AccessibleModuleCard, AccessibleLabExercise, AccessibleAssessment };