---
title: دستاویز بنائیں
sidebar_position: 2
---

# دستاویز بنائیں

دستاویزات **صفحات کے گروپس** ہیں جو درج ذیل کے ذریعے جڑے ہوتے ہیں:

- ایک **sidebar**
- **پچھلا/اگلا navigation**
- **versioning**

## اپنی پہلی Doc بنائیں

`docs/hello.md` پر ایک Markdown فائل بنائیں:

```md title="docs/hello.md"
# Hello

This is my **first Docusaurus document**!
```

ایک نئی دستاویز اب [http://localhost:3000/docs/hello](http://localhost:3000/docs/hello) پر دستیاب ہے۔

## Sidebar کو Configure کریں

Docusaurus خود بخود `docs` فولڈر سے **sidebar بناتا ہے**۔

Sidebar label اور position کو customize کرنے کے لیے metadata شامل کریں:

```md title="docs/hello.md" {1-4}
---
sidebar_label: 'Hi!'
sidebar_position: 3
---

# Hello

This is my **first Docusaurus document**!
```

آپ `sidebars.js` میں اپنا sidebar واضح طور پر بھی بنا سکتے ہیں:

```js title="sidebars.js"
export default {
  tutorialSidebar: [
    'intro',
    // highlight-next-line
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
};
```
