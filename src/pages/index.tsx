import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Translate from '@docusaurus/Translate';
import Hero from '@site/src/components/Hero';

export default function Home(): JSX.Element {
  return (
    <Layout
      title="Home"
      description="A comprehensive AI-native textbook for embodied intelligence and robotics">
      <Hero />
    </Layout>
  );
}