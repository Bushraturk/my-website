---
title: صفحہ بنائیں
sidebar_position: 1
---

# صفحہ بنائیں

**Markdown یا React** فائلز کو `src/pages` میں شامل کریں تاکہ ایک **standalone صفحہ** بنائیں:

- `src/pages/index.js` → `localhost:3000/`
- `src/pages/foo.md` → `localhost:3000/foo`
- `src/pages/foo/bar.js` → `localhost:3000/foo/bar`

## اپنا پہلا React صفحہ بنائیں

`src/pages/my-react-page.js` پر ایک فائل بنائیں:

```jsx title="src/pages/my-react-page.js"
import React from 'react';
import Layout from '@theme/Layout';

export default function MyReactPage() {
  return (
    <Layout>
      <h1>My React page</h1>
      <p>This is a React page</p>
    </Layout>
  );
}
```

ایک نیا صفحہ اب [http://localhost:3000/my-react-page](http://localhost:3000/my-react-page) پر دستیاب ہے۔

## اپنا پہلا Markdown صفحہ بنائیں

`src/pages/my-markdown-page.md` پر ایک فائل بنائیں:

```mdx title="src/pages/my-markdown-page.md"
# My Markdown page

This is a Markdown page
```

ایک نیا صفحہ اب [http://localhost:3000/my-markdown-page](http://localhost:3000/my-markdown-page) پر دستیاب ہے۔
