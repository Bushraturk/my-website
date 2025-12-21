import React from 'react';
import { useAuth } from '../contexts/AuthContext';

interface DifficultyIndicatorProps {
  level: 'beginner' | 'intermediate' | 'advanced';
  userLevel?: 'beginner' | 'intermediate' | 'advanced'; // Will default to user's level if not provided
}

const DifficultyIndicator: React.FC<DifficultyIndicatorProps> = ({ 
  level, 
  userLevel 
}) => {
  const { user, loading } = useAuth();
  
  // Get user's level from context if not provided
  const actualUserLevel = userLevel || (user ? user.background.programmingExperience : null);
  
  if (loading) {
    return <span className="difficulty-indicator">Loading...</span>;
  }

  // Determine if the content matches user's level
  const isAppropriate = actualUserLevel === level;
  const isTooEasy = 
    (level === 'beginner' && ['intermediate', 'advanced'].includes(actualUserLevel || '')) ||
    (level === 'intermediate' && actualUserLevel === 'advanced');
  const isTooHard = 
    (level === 'advanced' && ['beginner', 'intermediate'].includes(actualUserLevel || '')) ||
    (level === 'intermediate' && actualUserLevel === 'beginner');
  
  // Define CSS classes based on difficulty match
  let className = 'difficulty-indicator';
  if (isAppropriate) className += ' difficulty-appropriate';
  if (isTooEasy) className += ' difficulty-too-easy';
  if (isTooHard) className += ' difficulty-too-hard';
  
  // Capitalize the level for display
  const displayLevel = level.charAt(0).toUpperCase() + level.slice(1);
  
  return (
    <span className={className}>
      {displayLevel} 
      {isAppropriate ? ' (Recommended for you)' : 
       isTooEasy ? ' (Too easy for you)' : 
       isTooHard ? ' (Challenging for you)' : ''}
    </span>
  );
};

export default DifficultyIndicator;

// Add styling via CSS classes
