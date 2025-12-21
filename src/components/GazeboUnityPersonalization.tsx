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

  const { programmingExperience, hardwareExperience, roboticsBackground, interests } = userData.background;

  const getBonusTips = () => {
    const tips: { icon: string; title: string; content: string }[] = [];

    // Programming experience tips for Gazebo/Unity
    if (programmingExperience === 'beginner') {
      tips.push({
        icon: '🎮',
        title: 'Simulation Basics',
        content: 'Start with Gazebo\'s built-in tutorials. Focus on understanding world files and models before writing code.'
      });
    } else if (programmingExperience === 'intermediate') {
      tips.push({
        icon: '🔨',
        title: 'Plugin Development',
        content: 'Try creating custom Gazebo plugins! You can simulate sensors and actuators with your own logic.'
      });
    } else if (programmingExperience === 'advanced') {
      tips.push({
        icon: '⚡',
        title: 'Unity ML-Agents',
        content: 'Explore Unity ML-Agents for training RL policies directly in simulation. Great for advanced robotics research!'
      });
    }

    // Hardware experience tips
    if (hardwareExperience === 'none' || hardwareExperience === 'basic') {
      tips.push({
        icon: '🛡️',
        title: 'Safe Learning',
        content: 'Simulation is perfect for you! No risk of damaging hardware while learning robot control.'
      });
    } else {
      tips.push({
        icon: '🔄',
        title: 'Sim-to-Real',
        content: 'Focus on domain randomization and sim-to-real transfer techniques for deploying to real robots.'
      });
    }

    // Interest-based tips
    if (interests.includes('Simulation')) {
      tips.push({
        icon: '🌐',
        title: 'Digital Twin Expert',
        content: 'Perfect interest match! Learn to create accurate digital twins with physics and sensor simulation.'
      });
    }
    if (interests.includes('ROS 2')) {
      tips.push({
        icon: '🔗',
        title: 'ROS 2 + Gazebo',
        content: 'Use ros2_gz_bridge to connect ROS 2 nodes with Gazebo simulation seamlessly.'
      });
    }
    if (interests.includes('RL')) {
      tips.push({
        icon: '🧠',
        title: 'RL Training',
        content: 'Gazebo and Unity are great for RL! Check out OpenAI Gym wrappers for robot environments.'
      });
    }

    return tips;
  };

  const bonusTips = getBonusTips();

  return (
    <div className="personalization-content" style={{
      background: 'linear-gradient(135deg, #2d1f4e 0%, #1a1333 100%)',
      borderRadius: '12px',
      padding: '1.5rem',
      marginBottom: '2rem',
      border: '2px solid #FFD700'
    }}>
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

      <p style={{ color: '#b19cd9', marginBottom: '1rem' }}>
        <Translate id="personalization.simulationTips">Simulation tips tailored for you:</Translate>
      </p>

      <div style={{
        background: 'rgba(255,255,255,0.1)',
        borderRadius: '8px',
        padding: '1rem',
        marginBottom: '1rem'
      }}>
        <div style={{ display: 'flex', flexWrap: 'wrap', gap: '0.5rem' }}>
          <span style={{
            background: '#9C27B0',
            color: '#fff',
            padding: '0.25rem 0.75rem',
            borderRadius: '15px',
            fontSize: '0.85rem'
          }}>
            🎮 {programmingExperience} programmer
          </span>
          <span style={{
            background: '#673AB7',
            color: '#fff',
            padding: '0.25rem 0.75rem',
            borderRadius: '15px',
            fontSize: '0.85rem'
          }}>
            {hardwareExperience === 'none' ? '🖥️ Sim-focused' : '🔧 Hardware ready'}
          </span>
        </div>
      </div>

      <h4 style={{ color: '#FFD700', marginBottom: '0.75rem' }}>
        <Translate id="personalization.bonusTips">Your Bonus Tips</Translate>
      </h4>
      <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
        {bonusTips.map((tip, idx) => (
          <div key={idx} style={{
            background: 'rgba(255,255,255,0.1)',
            borderRadius: '8px',
            padding: '1rem',
            borderLeft: '4px solid #9C27B0'
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

const GazeboUnityPersonalization = () => {
  return (
    <BrowserOnly fallback={<div>Loading...</div>}>
      {() => <PersonalizationContent />}
    </BrowserOnly>
  );
};

export default GazeboUnityPersonalization;
