import React from 'react';
import { useAuth } from '../contexts/AuthContext';

const OnboardingContent: React.FC = () => {
  const { user, loading } = useAuth();

  if (loading || !user) {
    return null;
  }

  const { background } = user;
  
  return (
    <div className="onboarding-content">
      <h2>Recommended for You</h2>
      
      {/* Content based on interests */}
      {background.interests.includes('Simulation') && (
        <div className="recommendation">
          <h3>Recommended Module: Gazebo/Unity</h3>
          <p>Based on your interest in simulation, we recommend starting with the Gazebo/Unity module to build your digital twin skills.</p>
          <a href="/docs/gazebo-unity/intro" className="button button--primary">Start Module</a>
        </div>
      )}
      
      {background.interests.includes('ROS 2') && (
        <div className="recommendation">
          <h3>Recommended Module: ROS 2</h3>
          <p>Since you're interested in ROS 2, we suggest reviewing the advanced ROS 2 concepts in weeks 2-3.</p>
          <a href="/docs/ros2/week3" className="button button--primary">Start Module</a>
        </div>
      )}
      
      {background.interests.includes('NVIDIA Isaac') && (
        <div className="recommendation">
          <h3>Recommended Module: NVIDIA Isaac</h3>
          <p>Your interest in NVIDIA Isaac suggests you'd benefit from our AI-robot brain content.</p>
          <a href="/docs/nvidia-isaac/intro" className="button button--primary">Start Module</a>
        </div>
      )}
      
      {background.interests.includes('Humanoids') && (
        <div className="recommendation">
          <h3>Recommended Module: Vision-Language-Action</h3>
          <p>With your focus on humanoids, the VLA module will show you how to integrate vision, language, and action planning.</p>
          <a href="/docs/vla/intro" className="button button--primary">Start Module</a>
        </div>
      )}
      
      {/* Content based on experience level */}
      {background.programmingExperience === 'beginner' && (
        <div className="recommendation">
          <h3>Getting Started with Robotics Programming</h3>
          <p>Since you're beginning your programming journey, we recommend our foundational content with extra code explanations and step-by-step guides.</p>
          <p>Check out our <a href="/docs/ros2/week1-2">ROS 2 Introduction</a> for beginners.</p>
        </div>
      )}
      
      {background.hardwareExperience === 'none' && (
        <div className="recommendation">
          <h3>Simulation-First Approach</h3>
          <p>Since you're new to hardware, we recommend starting with simulation exercises before moving to physical robots.</p>
          <p>Try our <a href="/docs/gazebo-unity/intro">Gazebo/Unity module</a> to get hands-on experience in a virtual environment.</p>
        </div>
      )}
      
      {/* If no specific recommendations */}
      {background.interests.length === 0 && background.programmingExperience !== 'beginner' && background.hardwareExperience !== 'none' && (
        <div className="recommendation">
          <h3>Explore Our Content</h3>
          <p>Based on your experience, you're ready for advanced content. Explore any module that interests you!</p>
          <a href="/docs/intro" className="button button--primary">View All Modules</a>
        </div>
      )}
    </div>
  );
};

export default OnboardingContent;

// Add styling via CSS classes
