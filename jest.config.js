// jest.config.js
// Jest configuration for testing the Physical AI & Humanoid Robotics textbook

module.exports = {
  // The test environment to use for testing
  testEnvironment: 'jsdom',
  
  // Automatically clear mock calls and instances between every test
  clearMocks: true,
  
  // An array of glob patterns indicating a set of files for which coverage information should be collected
  collectCoverageFrom: [
    'src/**/*.{js,jsx,ts,tsx}',
    '!src/**/*.d.ts',
    '!src/**/node_modules/**',
  ],
  
  // The directory where Jest should output its coverage files
  coverageDirectory: 'coverage',
  
  // An array of regexp pattern strings used to skip coverage collection
  coveragePathIgnorePatterns: [
    '\\\\node_modules\\\\',
  ],
  
  // Indicates which provider should be used to instrument code for coverage
  coverageReporters: [
    'json',
    'text',
    'lcov',
    'clover',
  ],
  
  // An object that configures minimum threshold enforcement for coverage results
  coverageThreshold: {
    global: {
      branches: 80,
      functions: 85,
      lines: 85,
      statements: 85,
    },
  },
  
  // A map from regular expressions to module names or to arrays of module names that allow to stub out resources with a single module
  moduleNameMapper: {
    '\\.(css|less|scss|sass)$': 'identity-obj-proxy',
    '\\.(jpg|jpeg|png|gif|eot|otf|webp|svg|ttf|woff|woff2|mp4|webm|wav|mp3|m4a|aac|oga)$': '<rootDir>/__mocks__/fileMock.js',
  },
  
  // An array of regexp pattern strings that are matched against all test paths, matched tests are skipped
  testPathIgnorePatterns: [
    '\\\\node_modules\\\\',
  ],
  
  // This option allows the use of a custom results processor
  testResultsProcessor: undefined,
  
  // This option allows use of a custom test runner
  testRunner: 'jest-circus/runner',
  
  // Transform
  transform: {
    '^.+\\.(js|jsx|mjs|cjs|ts|tsx)$': '<rootDir>/node_modules/babel-jest',
    '^.+\\.css$': '<rootDir>/config/jest/cssTransform.js',
    '^(?!.*\\.(js|jsx|mjs|cjs|ts|tsx|css|json)$)': '<rootDir>/config/jest/fileTransform.js',
  },
  
  // TransformIgnorePatterns
  transformIgnorePatterns: [
    '[/\\\\]node_modules[/\\\\].+\\.(js|jsx|mjs|cjs|ts|tsx)$',
    '^.+\\.module\\.(css|sass|scss)$',
  ],
  
  // A preset that is used as a base for Jest's configuration
  preset: undefined,
  
  // A list of paths to directories that Jest should use to search for files in
  roots: [
    '<rootDir>/src',
    '<rootDir>/__tests__',
  ],
  
  // A list of paths to modules that run some code to configure or set up the testing framework
  setupFilesAfterEnv: ['<rootDir>/src/setupTests.ts'],
  
  // The glob patterns Jest uses to detect test files
  testMatch: [
    '<rootDir>/src/**/__tests__/**/*.{js,jsx,ts,tsx}',
    '<rootDir>/src/**/*.{spec,test}.{js,jsx,ts,tsx}',
    '<rootDir>/__tests__/**/*.{js,jsx,ts,tsx}',
  ],
  
  // An array of regexp pattern strings that are matched against all source file paths, matched files will skip transformation
  transformIgnorePatterns: [
    '<rootDir>/node_modules/(?!(react-dnd|@react-dnd|dnd-core|react-dnd-html5-backend)/)',
  ],
  
  // An array of regexp pattern strings that are matched against all modules before the module loader will automatically return a mock for them
  unmockedModulePathPatterns: undefined,
  
  // Indicates whether each individual test should be reported during the run
  verbose: true,
  
  // Whether to use watchman for file crawling
  watchman: true,
};