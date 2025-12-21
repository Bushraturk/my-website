// webpack.performance.config.js
// Additional Webpack configuration for performance optimization

module.exports = {
  optimization: {
    splitChunks: {
      chunks: 'all',
      cacheGroups: {
        // Separate vendor libraries from application code
        vendor: {
          test: /[\\/]node_modules[\\/]/,
          name: 'vendors',
          chunks: 'all',
        },
        // Separate common components used across multiple pages
        components: {
          test: /[\\/]src[\\/]components[\\/]/,
          name: 'components',
          chunks: 'all',
          priority: 10,
        },
      },
    },
    // Enable minimization
    minimize: true,
  },
  // Performance hints
  performance: {
    maxAssetSize: 250000,  // 250KB
    maxEntrypointSize: 250000,  // 250KB
    hints: 'warning',  // 'error' to enforce strict limits
  },
  // Additional resolve options for faster builds
  resolve: {
    extensions: ['.js', '.jsx', '.ts', '.tsx'],
  },
  // Enable source maps for debugging
  devtool: 'source-map',
};