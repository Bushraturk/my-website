import React from 'react';
import { useAuth } from '../contexts/AuthContext';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

interface PersonalizedContentProps {
  children: React.ReactNode;
  forProgrammingLevel?: 'beginner' | 'intermediate' | 'advanced';
  forHardwareLevel?: 'none' | 'basic' | 'intermediate' | 'advanced';
  forRoboticsBackground?: 'none' | 'academic' | 'industry';
  forInterest?: string; // Specific interest like 'ROS 2', 'Simulation', etc.
  hideIf?: boolean; // Condition to hide content
}

const PersonalizedContent: React.FC<PersonalizedContentProps> = ({
  children,
  forProgrammingLevel,
  forHardwareLevel,
  forRoboticsBackground,
  forInterest,
  hideIf = false
}) => {
  // Only use auth context in browser environment to avoid SSR issues
  if (!ExecutionEnvironment.canUseDOM) {
    // During SSR, just render children without personalization
    return <>{children}</>;
  }

  // In browser environment, use the auth context
  const { user, loading } = useAuth();

  if (loading) {
    // While loading, don't show personalized content
    return null;
  }

  // If user is not authenticated, show content to everyone
  if (!user) {
    return <>{children}</>;
  }

  // Check if content should be hidden based on user preferences
  if (hideIf) {
    return null;
  }

  // Check programming experience filter
  if (forProgrammingLevel && user.background.programmingExperience !== forProgrammingLevel) {
    return null;
  }

  // Check hardware experience filter
  if (forHardwareLevel && user.background.hardwareExperience !== forHardwareLevel) {
    return null;
  }

  // Check robotics background filter
  if (forRoboticsBackground && user.background.roboticsBackground !== forRoboticsBackground) {
    return null;
  }

  // Check interest filter
  if (forInterest && !user.background.interests.includes(forInterest)) {
    return null;
  }

  // If all conditions pass, show the content
  return <>{children}</>;
};

export default PersonalizedContent;
