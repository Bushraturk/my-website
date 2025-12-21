// __tests__/components/ModuleCard.test.tsx
// Unit tests for the ModuleCard component

import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import '@testing-library/jest-dom';
import { ModuleCard } from '../../src/components/ModuleCard';

describe('ModuleCard Component', () => {
  const defaultProps = {
    title: 'ROS 2 Fundamentals',
    description: 'Learn the basics of ROS 2',
    difficulty: 'Intermediate',
    weeks: 'Weeks 1-3',
    progress: 45,
    link: '/docs/ros2/intro',
  };

  it('renders correctly with required props', () => {
    render(<ModuleCard {...defaultProps} />);
    
    expect(screen.getByText('ROS 2 Fundamentals')).toBeInTheDocument();
    expect(screen.getByText('Learn the basics of ROS 2')).toBeInTheDocument();
    expect(screen.getByText('Intermediate')).toBeInTheDocument();
    expect(screen.getByText('Weeks 1-3')).toBeInTheDocument();
    
    // Check for progress indicator if user is logged in
    expect(screen.getByText('45%')).toBeInTheDocument();
  });

  it('allows users to navigate to the module', () => {
    render(<ModuleCard {...defaultProps} />);
    
    const linkElement = screen.getByRole('link', { name: /ROS 2 Fundamentals/i });
    expect(linkElement).toHaveAttribute('href', '/docs/ros2/intro');
  });

  it('displays correct progress percentage', () => {
    render(<ModuleCard {...defaultProps} progress={75} />);
    
    expect(screen.getByText('75%')).toBeInTheDocument();
  });

  it('shows appropriate difficulty indicator', () => {
    const { rerender } = render(<ModuleCard {...defaultProps} difficulty="Beginner" />);
    
    expect(screen.getByText('Beginner')).toBeInTheDocument();
    
    rerender(<ModuleCard {...defaultProps} difficulty="Advanced" />);
    expect(screen.getByText('Advanced')).toBeInTheDocument();
  });

  it('applies correct CSS classes for styling', () => {
    render(<ModuleCard {...defaultProps} />);
    
    const cardElement = screen.getByTestId('module-card');
    expect(cardElement).toHaveClass('module-card');
  });
});