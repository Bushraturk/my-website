import React from 'react';
import Layout from '@theme/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';
import Translate from '@docusaurus/Translate';
import useBaseUrl from '@docusaurus/useBaseUrl';

const UserProfilePageContent = () => {
  const [userData, setUserData] = React.useState(null);
  const [loading, setLoading] = React.useState(true);
  const homeUrl = useBaseUrl('/');

  React.useEffect(() => {
    // Check authentication state from localStorage
    if (typeof window !== 'undefined') {
      const mockUserData = localStorage.getItem('mockUser');
      if (mockUserData) {
        try {
          const parsedData = JSON.parse(mockUserData);
          setUserData(parsedData);
        } catch (e) {
          console.error('Error parsing user data:', e);
        }
      }
    }
    setLoading(false);
  }, []);

  // If no user data is found (not authenticated), show a message
  if (loading) {
    return (
      <Layout title="User Profile" description="User profile page">
        <div style={{ padding: '2rem' }}>
          <h1><Translate id="profile.loading">Loading...</Translate></h1>
        </div>
      </Layout>
    );
  }

  if (!userData) {
    return (
      <Layout title="User Profile" description="User profile page">
        <div style={{ padding: '2rem' }}>
          <h1><Translate id="profile.pleaseSignIn">Please sign in to view your profile</Translate></h1>
          <p><Translate id="profile.needLogin">You need to be logged in to access your profile information.</Translate></p>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="User Profile" description="User profile page">
      <div style={{ padding: '2rem' }}>
        <div className="container">
          <h1><Translate id="profile.title">User Profile</Translate></h1>

          <h2>
            <Translate id="profile.welcome">Welcome</Translate>, {userData.user?.name || userData.user?.email || "User"}!
          </h2>

          <div className="profile-info">
            <h3><Translate id="profile.yourInfo">Your Information</Translate></h3>
            <ul>
              <li><strong><Translate id="profile.email">Email</Translate>:</strong> {userData.user?.email}</li>
              <li><strong><Translate id="profile.programmingExp">Programming Experience</Translate>:</strong> {userData.background?.programmingExperience || "Not specified"}</li>
              <li><strong><Translate id="profile.hardwareExp">Hardware Experience</Translate>:</strong> {userData.background?.hardwareExperience || "Not specified"}</li>
              <li><strong><Translate id="profile.roboticsBackground">Robotics Background</Translate>:</strong> {userData.background?.roboticsBackground || "Not specified"}</li>
              <li>
                <strong><Translate id="profile.interests">Interests</Translate>:</strong> {userData.background?.interests && Array.isArray(userData.background.interests)
                  ? userData.background.interests.join(', ')
                  : userData.background?.interests || "Not specified"}
              </li>
            </ul>
          </div>

          <div className="personalized-content">
            <h3><Translate id="profile.personalizedContent">Personalized Content</Translate></h3>
            <p><Translate id="profile.personalizedDesc">Based on your background, we're showing you content that matches your experience level and interests.</Translate></p>
          </div>

          <div className="homepage-redirect">
            <h3><Translate id="profile.continueLearning">Continue Learning</Translate></h3>
            <a href={homeUrl} className="button button--primary">
              <Translate id="profile.goHome">Go to Homepage</Translate>
            </a>
          </div>
        </div>
      </div>
    </Layout>
  );
};

const UserProfilePage = () => {
  return (
    <BrowserOnly>
      {() => <UserProfilePageContent />}
    </BrowserOnly>
  );
};

export default UserProfilePage;
