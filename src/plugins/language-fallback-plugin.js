// src/plugins/language-fallback-plugin.js
// Plugin to handle language switching with fallbacks when pages don't exist in target language

module.exports = function plugin(context, options) {
  return {
    name: 'language-fallback-plugin',

    // Add a client module to handle language switching with fallbacks
    getClientModules() {
      return [
        require.resolve('./languageswitcher'),
      ];
    },
  };
};
