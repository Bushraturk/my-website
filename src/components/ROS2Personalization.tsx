import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import Translate from '@docusaurus/Translate';
import Link from '@docusaurus/Link';

interface UserBackground {
  programmingExperience: string;
  hardwareExperience: string;
  roboticsBackground: string;
  interests: string[];
}

interface UserData {
  email: string;
  background: UserBackground;
}

const PersonalizationContent = () => {
  const [userData, setUserData] = useState<UserData | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const mockUserData = localStorage.getItem('mockUser');
    if (mockUserData) {
      try {
        const parsedData = JSON.parse(mockUserData);
        // Normalize interests to array
        const interests = Array.isArray(parsedData.background?.interests)
          ? parsedData.background.interests
          : (parsedData.background?.interests || parsedData.interests
            ? String(parsedData.background?.interests || parsedData.interests).split(',').map(i => i.trim())
            : []);

        setUserData({
          email: parsedData.email,
          background: {
            programmingExperience: parsedData.background?.programmingExperience || parsedData.programmingExperience || 'beginner',
            hardwareExperience: parsedData.background?.hardwareExperience || parsedData.hardwareExperience || 'none',
            roboticsBackground: parsedData.background?.roboticsBackground || parsedData.roboticsBackground || 'none',
            interests: interests
          }
        });
      } catch (e) {
        console.error('Error parsing user data:', e);
      }
    }
    setLoading(false);
  }, []);

  if (loading) {
    return (
      <div className="personalization-loading">
        <Translate id="personalization.loading">Loading personalized content...</Translate>
      </div>
    );
  }

  // If not logged in, show signup prompt
  if (!userData) {
    return (
      <div className="personalization-prompt" style={{
        background: 'linear-gradient(135deg, #1a1a2e 0%, #16213e 100%)',
        borderRadius: '12px',
        padding: '2rem',
        marginBottom: '2rem',
        border: '2px solid #FFD700'
      }}>
        <h3 style={{ color: '#FFD700', marginTop: 0 }}>
          <Translate id="personalization.unlockTitle">Unlock Personalized Learning!</Translate>
        </h3>
        <p style={{ color: '#ddd' }}>
          <Translate id="personalization.unlockDesc">
            Sign up to get personalized tips, bonus content, and recommendations based on your experience level and interests.
          </Translate>
        </p>
        <ul style={{ color: '#ccc', textAlign: 'left' }}>
          <li><Translate id="personalization.benefit1">Bonus tips tailored to your skill level</Translate></li>
          <li><Translate id="personalization.benefit2">Personalized learning path recommendations</Translate></li>
          <li><Translate id="personalization.benefit3">Extra resources based on your interests</Translate></li>
        </ul>
        <Link
          to="/signup"
          style={{
            background: 'linear-gradient(90deg, #FFD700, #FFA500)',
            color: '#000',
            padding: '0.75rem 1.5rem',
            borderRadius: '6px',
            fontWeight: 'bold',
            textDecoration: 'none',
            display: 'inline-block',
            marginTop: '1rem'
          }}
        >
          <Translate id="personalization.signupBtn">Sign Up for Free</Translate>
        </Link>
      </div>
    );
  }

  // Generate personalized content based on user background
  const { programmingExperience, hardwareExperience, roboticsBackground, interests } = userData.background;

  // Bonus tips based on experience level
  const getBonusTips = () => {
    const tips: { icon: string; title: string; content: string }[] = [];

    // Programming experience tips
    if (programmingExperience === 'beginner') {
      tips.push({
        icon: '💡',
        title: 'Beginner Tip',
        content: 'Start with Python basics before diving into ROS 2. Focus on understanding variables, functions, and classes first.'
      });
    } else if (programmingExperience === 'intermediate') {
      tips.push({
        icon: '🚀',
        title: 'Level Up Tip',
        content: 'Since you have programming experience, try creating custom message types and explore the rclpy API deeper.'
      });
    } else if (programmingExperience === 'advanced') {
      tips.push({
        icon: '⚡',
        title: 'Advanced Challenge',
        content: 'Consider contributing to ROS 2 packages or creating your own! Check out ros2/rclcpp for C++ implementations.'
      });
    }

    // Hardware experience tips
    if (hardwareExperience === 'none' || hardwareExperience === 'basic') {
      tips.push({
        icon: '🔧',
        title: 'Hardware Tip',
        content: 'Use simulation first! Gazebo is great for testing without physical hardware. You can learn concepts safely.'
      });
    } else {
      tips.push({
        icon: '🤖',
        title: 'Hardware Integration',
        content: 'Great hardware experience! You can skip ahead to hardware integration sections and real robot examples.'
      });
    }

    // Robotics background tips
    if (roboticsBackground === 'academic') {
      tips.push({
        icon: '📚',
        title: 'Academic Focus',
        content: 'Check out the research papers section and ROS 2 design documents for deeper theoretical understanding.'
      });
    } else if (roboticsBackground === 'industry') {
      tips.push({
        icon: '🏭',
        title: 'Industry Ready',
        content: 'Focus on ROS 2 lifecycle nodes, quality of service settings, and real-time capabilities for production use.'
      });
    }

    // Interest-based tips
    if (interests.includes('ROS 2')) {
      tips.push({
        icon: '🎯',
        title: 'ROS 2 Deep Dive',
        content: 'Since ROS 2 is your interest, explore advanced topics like DDS configuration and custom middleware.'
      });
    }
    if (interests.includes('Simulation')) {
      tips.push({
        icon: '🎮',
        title: 'Simulation Focus',
        content: 'Check out our Gazebo/Unity module next! It integrates perfectly with ROS 2 for testing.'
      });
    }
    if (interests.includes('NVIDIA Isaac')) {
      tips.push({
        icon: '🧠',
        title: 'AI Integration',
        content: 'After ROS 2 basics, jump to NVIDIA Isaac module for AI-powered perception and control.'
      });
    }

    return tips;
  };

  const bonusTips = getBonusTips();

  return (
    <div className="personalization-content" style={{
      background: 'linear-gradient(135deg, #0d4f0d 0%, #1a3a1a 100%)',
      borderRadius: '12px',
      padding: '1.5rem',
      marginBottom: '2rem',
      border: '2px solid #FFD700'
    }}>
      {/* Bonus Points Badge */}
      <div style={{
        display: 'flex',
        alignItems: 'center',
        gap: '0.5rem',
        marginBottom: '1rem',
        background: '#FFD700',
        color: '#000',
        padding: '0.5rem 1rem',
        borderRadius: '20px',
        width: 'fit-content',
        fontWeight: 'bold'
      }}>
        <span>🎁</span>
        <span>+{bonusTips.length * 10} <Translate id="personalization.bonusPoints">Bonus Points</Translate></span>
      </div>

      <h3 style={{ color: '#FFD700', marginTop: 0 }}>
        <Translate id="personalization.welcomeBack">Welcome Back!</Translate>
        <span style={{ color: '#fff' }}> {userData.email.split('@')[0]}</span>
      </h3>

      <p style={{ color: '#90EE90', marginBottom: '1rem' }}>
        <Translate id="personalization.personalizedFor">Personalized content based on your profile:</Translate>
      </p>

      {/* User Profile Summary */}
      <div style={{
        background: 'rgba(255,255,255,0.1)',
        borderRadius: '8px',
        padding: '1rem',
        marginBottom: '1rem'
      }}>
        <div style={{ display: 'flex', flexWrap: 'wrap', gap: '0.5rem' }}>
          <span style={{
            background: '#FFD700',
            color: '#000',
            padding: '0.25rem 0.75rem',
            borderRadius: '15px',
            fontSize: '0.85rem'
          }}>
            {programmingExperience === 'beginner' ? '🌱 Beginner' :
             programmingExperience === 'intermediate' ? '🌿 Intermediate' : '🌳 Advanced'}
          </span>
          <span style={{
            background: '#4CAF50',
            color: '#fff',
            padding: '0.25rem 0.75rem',
            borderRadius: '15px',
            fontSize: '0.85rem'
          }}>
            {roboticsBackground === 'none' ? '🆕 New to Robotics' :
             roboticsBackground === 'academic' ? '📚 Academic' : '🏭 Industry'}
          </span>
          {interests.map((interest, idx) => (
            <span key={idx} style={{
              background: '#2196F3',
              color: '#fff',
              padding: '0.25rem 0.75rem',
              borderRadius: '15px',
              fontSize: '0.85rem'
            }}>
              {interest}
            </span>
          ))}
        </div>
      </div>

      {/* Bonus Tips */}
      <h4 style={{ color: '#FFD700', marginBottom: '0.75rem' }}>
        <Translate id="personalization.bonusTips">Your Bonus Tips</Translate>
      </h4>
      <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
        {bonusTips.map((tip, idx) => (
          <div key={idx} style={{
            background: 'rgba(255,255,255,0.1)',
            borderRadius: '8px',
            padding: '1rem',
            borderLeft: '4px solid #FFD700'
          }}>
            <strong style={{ color: '#FFD700' }}>{tip.icon} {tip.title}</strong>
            <p style={{ color: '#ddd', margin: '0.5rem 0 0 0', fontSize: '0.95rem' }}>
              {tip.content}
            </p>
          </div>
        ))}
      </div>
    </div>
  );
};

const ROS2Personalization = () => {
  return (
    <BrowserOnly fallback={<div>Loading...</div>}>
      {() => <PersonalizationContent />}
    </BrowserOnly>
  );
};

export default ROS2Personalization;
