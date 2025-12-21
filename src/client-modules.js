import React from 'react';

// Return the element as-is without wrapping with AuthProvider
// This avoids infinite render loops during static site generation
export function wrapRootElement({ element }) {
  return element;
}