---
title: بلاگ پوسٹ بنائیں
sidebar_position: 3
---

# بلاگ پوسٹ بنائیں

Docusaurus **ہر بلاگ پوسٹ کے لیے ایک صفحہ** بناتا ہے، لیکن ساتھ ہی ایک **blog index page**، ایک **tag system**، ایک **RSS** feed بھی...

## اپنی پہلی Post بنائیں

`blog/2021-02-28-greetings.md` پر ایک فائل بنائیں:

```md title="blog/2021-02-28-greetings.md"
---
slug: greetings
title: Greetings!
authors:
  - name: Joel Marcey
    title: Co-creator of Docusaurus 1
    url: https://github.com/JoelMarcey
    image_url: https://github.com/JoelMarcey.png
  - name: Sébastien Lorber
    title: Docusaurus maintainer
    url: https://sebastienlorber.com
    image_url: https://github.com/slorber.png
tags: [greetings]
---

Congratulations, you have made your first post!

Feel free to play around and edit this post as much as you like.
```

ایک نئی بلاگ پوسٹ اب [http://localhost:3000/blog/greetings](http://localhost:3000/blog/greetings) پر دستیاب ہے۔
