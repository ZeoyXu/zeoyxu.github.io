---
title: 利用github和Jekyll搭建个人网站
date: 2024-04-04 11:00:00 +0800
categories: [Git]
tags: [网站, 工具]
toc: true 
comments: false
math: true
---

参考链接

[Jekyll官方网站](https://jekyllrb.com/)

[chirpy官方网站](https://chirpy.cotes.page/)

# 软件安装

## Git安装

见上一篇

## 安装Ruby

下载地址：[](https://rubyinstaller.org/downloads/)

选择Ruby+Devkit版本，一路确定后在弹窗中选择3

![](/assets/img/personalWeb1.png)

检查是否安装成功

```shell
ruby -v

gem -v
```

## 安装Jekyll

```shell
gem install jekyll bundler
```

检查安装

```shell
jekyll -v
```

# 搭建个人博客

## 参考Chirpy

打开([chirpy仓库](https://github.com/cotes2020/chirpy-starter))

在此基础上create a new repository，命名为yourname.github.io，git clone到本地

## 配置

在目录下执行

```shell
bundle
```

配置_config.yml文件：

```yml
# 网站设置

# 导入主题
theme: jekyll-theme-chirpy

# 站点的语言 › https://www.w3.org/International/questions/qa-choosing-language-tags
# 如果在文件夹 `_data/locales` 下存在同名文件，则布局语言也会改变，
# 否则，布局语言使用默认值 'en'。
lang: zh-CN

# 修改为你的时区 › https://kevinnovak.github.io/Time-Zone-Picker
timezone: Asia/Shanghai

# jekyll-seo-tag 设置 › https://github.com/jekyll/jekyll-seo-tag/blob/master/docs/usage.md
# --------------------------
title: Zeoy Xu       # 主标题

tagline: a student # 副标题

description: >- # 由 seo meta 和 atom feed 使用
  Zeoy Xu的个人博客

# 填入你的网站的协议和域名.
# 例如，'https://username.github.io'，注意不要以 '/' 结尾。
url: "https://zeoyxu.github.io"

author: ZeoyXu # 全名

github:
  username: ZeoyXu# Github 用户名

twitter:
  username: # 推特用户名

social:
  # 改为你的全名。
  # 这将会显示为帖子的默认作者以及页脚的版权所有者
  name: ZeoyXu
  email: zeoyxu@foxmail.com # 邮箱地址
  links:
    # 第一个链接将会作为版权所有者的链接
    - # https://
    - # https://# 推特主页
    - # https://# Github 主页
    # 取消注释以添加更多链接
    # - https://www.facebook.com/username
    # - https://www.linkedin.com/in/username

# 网站认证设置
webmaster_verifications:
  google: # 填入你的 Google 认证代码
  bing: # 填入你的 Bing 认证代码
  alexa: # 填入你的 Alexa 认证代码
  yandex: # 填入你的 Yandex 认证代码
  baidu: # 填入你的百度认证代码
  facebook: # 填入你的 Facebook 认证代码

# ↑ --------------------------
# `jekyll-seo-tag` 设置结束

# 网页分析设置
analytics:
  google:
    id: # 填入你的 Google Analytics ID
  goatcounter:
    id: # 填入你的 GoatCounter ID

# 页面浏览量设置
pageviews:
  provider: # 目前仅支持 'goatcounter'

# 设置首选配色方案
#
# 提示: 保留空白会跟随系统的默认首选配色，并且在侧边栏的底部会出现
# 一个可以转换亮色主题和暗色主题的按钮。
#
# 可用选项:
#
#     light  - 使用亮色方案
#     dark   - 使用暗色方案
#
theme_mode: # [light | dark]

# 启用动画背景特效
backgroud_animation: false

# 启用鼠标点击特效
mouse_click_effect: false

# 图像 CDN 端点。
# 注意一旦被分配，CDN 网址会被加在
# 所有以 '/' 开始的图像（网站头像和帖子中的图像）路径上
# 
# 例如，'https://cdn.com'
img_cdn: ""

# 侧边栏的头像，支持本地或 CROS 资源
avatar: /assets/xxxr.png

# SEO `og:image` 元标记中使用的全站社交预览图像的 URL。
# 可以在帖子的 Front Matter 中使用 `page.image` 覆盖。
social_preview_image: # 字符串，本地或跨域资源

# 设置显示右侧边栏的内容
panel:
  # 设置显示在帖子中右侧边栏的内容
  # 
  # 提示:
  #   全局开关为 false 时必定不显示对应块。
  #   全局开关为 true 时，可以在帖子的 md 文件头部使用形如:
  #      lastmod: false
  #   的方式为该帖子单独关闭右侧边栏的功能。
  post:
    # 最近更新
    lastmod: true
    # 热门标签
    trending_tags: false
    # 外部链接
    external_links: true
    # 目录
    toc: true

comments:
  # 帖子评论区全局开关，例如，'disqus'。保持为空表示禁用。
  provider:
  # provider 选项可以是如下：
  # disqus 评论系统 > https://disqus.com/
  disqus:
    shortname: # 填入你的 disqus shorname › https://help.disqus.com/en/articles/1717111-what-s-a-shortname
  # utterances 评论系统 › https://utteranc.es/
  utterances:
    repo: # <Github 用户名>/<仓库名>
    issue_term: # < 链接 | 路径 | 标题 | ...>
  # Waline 评论系统 › https://waline.js.org/
  waline:
    server: # Vercal 服务端地址
    placeholder: 说点什么吧！ # 空白评论框时显示的文字
    avatar: mp # 默认头像  › https://waline.js.org/guide/client/avatar.html
  # Giscus 选项 › https://giscus.app
  giscus:
    repo: # <Github 用户名>/<仓库名>
    repo_id:
    category:
    category_id:
    mapping: # 可选项，默认为 'pathname'
    strict: # 可选项，默认为 '0'
    input_position: # 可选项，默认为 'bottom'
    lang: # 可选项，默认为 `site.lang` 的值
    reactions_enabled: # 可选项，默认为 1

# 自托管静态资产，可选 › https://github.com/cotes2020/chirpy-static-assets
assets:
  self_host:
    enabled: # 布尔值, 置空表示否
    # 指定 Jekyll 环境，置空意味着都启用
    # 仅在 `assets.self_host.enabled` 为 'true' 时生效
    env: # [development|production]

pwa:
  enabled: true # PWA 特性选项（可安装）
  cache:
    enabled: true # PWA 离线缓存选项
    # 在此处定义不被 PWA 缓存的路径。
    # 通常其值是使用和当前网站相同域名的其他网站的 `baseurl`。
    deny_paths:
      # - "/example"  # 符合 `<SITE_URL>/example/*` 的 URL 不会被 PWA 缓存

# 每一页的帖子数量
paginate: 10

# 网站的基础 URL
baseurl: ""

# ------------ 下面的设置不建议修改 ------------------

kramdown:
  syntax_highlighter: rouge
  syntax_highlighter_opts: # Rouge 设置 › https://github.com/jneen/rouge#full-options
    css_class: highlight
    # default_lang: console
    span:
      line_numbers: false
    block:
      line_numbers: true
      start_line: 1

collections:
  tabs:
    output: true
    sort_by: order

defaults:
  - scope:
      path: "" # 空字符串意味着项目里的所有文件
      type: posts
    values:
      layout: post
      comments: true # 在帖子中启用评论
      # 右侧边栏的默认显示
      lastmod: true
      trending_tags: true
      external_links: true
      toc: true
      license: true
      # 不要改变它除非你是 Jekyll 以及 Web 开发的专家，
      # 或者你认为自己足够聪明可以在此模板中修改为其他的相对路径。
      permalink: /posts/:title/
  - scope:
      path: _drafts
    values:
      comments: false
  - scope:
      path: ""
      type: tabs # 参考 `site.collections`
    values:
      layout: page
      permalink: /:title/
  - scope:
      path: assets/js/dist
    values:
      swcache: true

sass:
  style: compressed

compress_html:
  clippings: all
  comments: all
  endings: all
  profile: false
  blanklines: false
  ignore:
    envs: [development]

exclude:
  - "*.gem"
  - "*.gemspec"
  - docs
  - tools
  - README.md
  - LICENSE
  - rollup.config.js
  - package*.json

jekyll-archives:
  enabled: [categories, tags]
  layouts:
    category: category
    tag: tag
  permalinks:
    tag: /tags/:name/
    category: /categories/:name/
```

在本地预览：

```shell
bundle exec jekyll serve
```

然后浏览器访问提示的网址

## 部署

部署到github前一定要看上面的url有没有改成你的域名！

github打开github.io的settings在Page里Source选择Github Actions

修改后提交:

```git
git add .
git commit -m "评论"
git push origin main
```

# 发布文章

详见：[Writing a New Post | Chirpy](https://chirpy.cotes.page/posts/write-a-new-post/)

在_posts文件夹下新建md文件，命名要求：YYYY-MM-DD-TITLE.md

在文章开始加上：

```markdown
---
title: 利用github和Jekyll搭建个人网站
date: 2024-04-04 11:00:00 +0800
categories: [Git]   # 目录，可以有子目录
tags: [网站, 工具]    
toc: true   # 是否开启文章目录
comments: false    # 是否开启评论
math: true  # 数学
---
```

写完后提交到github即可！
