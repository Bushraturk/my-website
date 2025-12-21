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
        border: '2px solid #76B900'
      }}>
        <h3 style={{ color: '#76B900', marginTop: 0 }}>
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
            background: 'linear-gradient(90deg, #76B900, #8BC34A)',
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

    // Programming experience tips for NVIDIA Isaac
    if (programmingExperience === 'beginner') {
      tips.push({
        icon: '🎯',
        title: 'Getting Started',
        content: 'Start with NVIDIA Isaac Sim tutorials. Focus on understanding the GUI before writing Python scripts.'
      });
    } else if (programmingExperience === 'intermediate') {
      tips.push({
        icon: '🔥',
        title: 'Custom Extensions',
        content: 'Create custom Isaac Sim extensions! Use the OmniGraph visual scripting for rapid prototyping.'
      });
    } else if (programmingExperience === 'advanced') {
      tips.push({
        icon: '⚡',
        title: 'CUDA Optimization',
        content: 'Leverage CUDA kernels for custom perception algorithms. Check out Isaac ROS for production deployment.'
      });
    }

    // Hardware experience tips
    if (hardwareExperience === 'none' || hardwareExperience === 'basic') {
      tips.push({
        icon: '💻',
        title: 'GPU Requirements',
        content: 'Isaac Sim needs RTX GPU. If you don\'t have one, use NVIDIA\'s cloud instances or focus on Isaac ROS.'
      });
    } else {
      tips.push({
        icon: '🤖',
        title: 'Jetson Deployment',
        content: 'Deploy trained models to NVIDIA Jetson! Great for edge AI robotics applications.'
      });
    }

    // Interest-based tips
    if (interests.includes('NVIDIA Isaac')) {
      tips.push({
        icon: '🏆',
        title: 'Perfect Match',
        content: 'Isaac is your interest! Explore Isaac Gym for RL training with thousands of parallel environments.'
      });
    }
    if (interests.includes('RL')) {
      tips.push({
        icon: '🧠',
        title: 'RL at Scale',
        content: 'Isaac Gym enables training RL policies 10000x faster than real-time. Perfect for your RL interest!'
      });
    }
    if (interests.includes('Humanoids')) {
      tips.push({
        icon: '🦿',
        title: 'Humanoid Robots',
        content: 'Isaac Sim has humanoid robot models. Check out the locomotion examples for bipedal walking!'
      });
    }

    // Robotics background tips
    if (roboticsBackground === 'academic') {
      tips.push({
        icon: '📖',
        title: 'Research Tools',
        content: 'Isaac Sim supports domain randomization and synthetic data generation for research projects.'
      });
    } else if (roboticsBackground === 'industry') {
      tips.push({
        icon: '🏭',
        title: 'Production Ready',
        content: 'Use Isaac ROS for production. It\'s designed for real-world industrial robotics deployment.'
      });
    }

    return tips;
  };

  const bonusTips = getBonusTips();

  return (
    <div className="personalization-content" style={{
      background: 'linear-gradient(135deg, #1a2f1a 0%, #0d1f0d 100%)',
      borderRadius: '12px',
      padding: '1.5rem',
      marginBottom: '2rem',
      border: '2px solid #76B900'
    }}>
      <div style={{
        display: 'flex',
        alignItems: 'center',
        gap: '0.5rem',
        marginBottom: '1rem',
        background: '#76B900',
        color: '#000',
        padding: '0.5rem 1rem',
        borderRadius: '20px',
        width: 'fit-content',
        fontWeight: 'bold'
      }}>
        <span>🎁</span>
        <span>+{bonusTips.length * 10} <Translate id="personalization.bonusPoints">Bonus Points</Translate></span>
      </div>

      <h3 style={{ color: '#76B900', marginTop: 0 }}>
        <Translate id="personalization.welcomeBack">Welcome Back!</Translate>
        <span style={{ color: '#fff' }}> {userData.email.split('@')[0]}</span>
      </h3>

      <p style={{ color: '#90EE90', marginBottom: '1rem' }}>
        <Translate id="personalization.aiTips">AI-powered tips for your learning journey:</Translate>
      </p>

      <div style={{
        background: 'rgba(255,255,255,0.1)',
        borderRadius: '8px',
        padding: '1rem',
        marginBottom: '1rem'
      }}>
        <div style={{ display: 'flex', flexWrap: 'wrap', gap: '0.5rem' }}>
          <span style={{
            background: '#76B900',
            color: '#000',
            padding: '0.25rem 0.75rem',
            borderRadius: '15px',
            fontSize: '0.85rem',
            fontWeight: 'bold'
          }}>
            🧠 {programmingExperience} level
          </span>
          <span style={{
            background: '#4CAF50',
            color: '#fff',
            padding: '0.25rem 0.75rem',
            borderRadius: '15px',
            fontSize: '0.85rem'
          }}>
            {hardwareExperience === 'advanced' || hardwareExperience === 'intermediate' ? '🎮 GPU Ready' : '☁️ Cloud Option'}
          </span>
        </div>
      </div>

      <h4 style={{ color: '#76B900', marginBottom: '0.75rem' }}>
        <Translate id="personalization.bonusTips">Your Bonus Tips</Translate>
      </h4>
      <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
        {bonusTips.map((tip, idx) => (
          <div key={idx} style={{
            background: 'rgba(255,255,255,0.1)',
            borderRadius: '8px',
            padding: '1rem',
            borderLeft: '4px solid #76B900'
          }}>
            <strong style={{ color: '#76B900' }}>{tip.icon} {tip.title}</strong>
            <p style={{ color: '#ddd', margin: '0.5rem 0 0 0', fontSize: '0.95rem' }}>
              {tip.content}
            </p>
          </div>
        ))}
      </div>
    </div>
  );
};

const NvidiaIsaacPersonalization = () => {
  return (
    <BrowserOnly fallback={<div>Loading...</div>}>
      {() => <PersonalizationContent />}
    </BrowserOnly>
  );
};

export default NvidiaIsaacPersonalization;
