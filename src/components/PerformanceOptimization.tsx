import React, { Suspense, lazy } from 'react';
import {useColorMode} from '@docusaurus/theme-common';

// Lazy load heavy components for better initial load performance
const LazyModuleCard = lazy(() => import('./ModuleCard'));
const LazyLabExercise = lazy(() => import('./LabExercise'));
const LazyAssessment = lazy(() => import('./Assessment'));
const LazyPersonalizedContent = lazy(() => import('./PersonalizedContent'));

// Performance optimized wrapper for components that might be heavy
interface PerformanceWrapperProps {
  children: React.ReactNode;
  fallback?: React.ReactNode;
  className?: string;
}

// A reusable wrapper component for performance optimization
const PerformanceWrapper: React.FC<PerformanceWrapperProps> = ({ 
  children, 
  fallback = <div>Loading content...</div>, 
  className = "" 
}) => {
  const { colorMode } = useColorMode();
  
  // Add performance-related attributes
  return (
    <div 
      className={className}
      data-color-mode={colorMode}
      style={{ contain: 'layout style paint' }} // CSS containment for performance
    >
      <Suspense fallback={fallback}>
        {children}
      </Suspense>
    </div>
  );
};

// Optimized component rendering with lazy loading and code splitting
const OptimizedModuleCard = (props: any) => (
  <PerformanceWrapper fallback={<div>Loading module...</div>}>
    <LazyModuleCard {...props} />
  </PerformanceWrapper>
);

const OptimizedLabExercise = (props: any) => (
  <PerformanceWrapper fallback={<div>Loading exercise...</div>}>
    <LazyLabExercise {...props} />
  </PerformanceWrapper>
);

const OptimizedAssessment = (props: any) => (
  <PerformanceWrapper fallback={<div>Loading assessment...</div>} className="assessment-container">
    <LazyAssessment {...props} />
  </PerformanceWrapper>
);

const OptimizedPersonalizedContent = (props: any) => (
  <PerformanceWrapper fallback={<div>Loading personalized content...</div>}>
    <LazyPersonalizedContent {...props} />
  </PerformanceWrapper>
);

// Memoized version of components to prevent unnecessary re-renders
const MemoizedModuleCard = React.memo(OptimizedModuleCard);
const MemoizedLabExercise = React.memo(OptimizedLabExercise);
const MemoizedAssessment = React.memo(OptimizedAssessment);
const MemoizedPersonalizedContent = React.memo(OptimizedPersonalizedContent);

export {
  PerformanceWrapper,
  OptimizedModuleCard,
  OptimizedLabExercise,
  OptimizedAssessment,
  OptimizedPersonalizedContent,
  MemoizedModuleCard,
  MemoizedLabExercise,
  MemoizedAssessment,
  MemoizedPersonalizedContent
};

// Additional performance utilities
export const useImagePreload = (src: string) => {
  React.useEffect(() => {
    const img = new Image();
    img.src = src;
  }, [src]);
};

// Debounce utility for performance
export const useDebounce = <T,>(value: T, delay: number): T => {
  const [debouncedValue, setDebouncedValue] = React.useState<T>(value);

  React.useEffect(() => {
    const handler = setTimeout(() => {
      setDebouncedValue(value);
    }, delay);

    return () => {
      clearTimeout(handler);
    };
  }, [value, delay]);

  return debouncedValue;
};