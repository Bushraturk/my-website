import React from 'react';

interface BonusPointsDisplayProps {
  points: number;
  message?: string;
}

const BonusPointsDisplay: React.FC<BonusPointsDisplayProps> = ({ 
  points = 50, 
  message = "Bonus points earned for personalizing content!" 
}) => {
  return (
    <div className="bonus-points-display">
      <div className="bonus-points-badge">
        <span className="bonus-icon">🎁</span>
        <span className="bonus-text">+{points} Bonus Points</span>
      </div>
      <p>{message}</p>
    </div>
  );
};

export default BonusPointsDisplay;