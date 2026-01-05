# Quickstart: Docusaurus Book Setup

## Prerequisites

- Node.js (version 18.0 or higher)
- npm or yarn package manager
- Git (optional, for version control)

## Setup Steps

### 1. Create a New Docusaurus Project

```bash
npx create-docusaurus@latest my-book-site classic
cd my-book-site
```

### 2. Install Dependencies

```bash
npm install
```

### 3. Start Development Server

```bash
npm start
```

This will start a local development server at http://localhost:3000 where you can preview your book.

### 4. Project Structure Overview

After setup, your project will have the following structure:

```
my-book-site/
├── blog/                 # Blog posts (optional)
├── docs/                 # Documentation files (chapters and lessons)
├── src/
│   ├── components/       # Custom React components
│   ├── css/              # Custom styles
│   └── pages/            # Custom pages
├── static/               # Static assets (images, files)
├── docusaurus.config.js  # Site configuration
├── sidebars.js           # Navigation sidebar configuration
└── package.json          # Dependencies and scripts
```

## Creating Your First Chapter

### 1. Create a Chapter Directory

```bash
mkdir docs/chapter-1
```

### 2. Add Chapter Metadata

Create `docs/chapter-1/_category_.json`:

```json
{
  "label": "Chapter 1: Introduction",
  "position": 1,
  "collapsible": true,
  "collapsed": false
}
```

### 3. Create Lesson Files

Create `docs/chapter-1/lesson-1.md`:

```markdown
---
id: lesson-1
title: Introduction to the Topic
sidebar_label: Lesson 1
sidebar_position: 1
description: Learn the basics of this topic
---

# Introduction to the Topic

This is the content of your first lesson.

## Section 1

Content for the first section.

## Section 2

Content for the second section.
```

### 4. Update Sidebar Configuration

Edit `sidebars.js` to include your chapter and lessons:

```javascript
module.exports = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Chapter 1: Introduction',
      items: ['chapter-1/lesson-1', 'chapter-1/lesson-2', 'chapter-1/lesson-3'],
    },
  ],
};
```

## Building for Production

To build the static assets for deployment:

```bash
npm run build
```

The built site will be available in the `build/` directory.

## Deployment Options

### GitHub Pages
1. Update `docusaurus.config.js` with your GitHub repository details
2. Run `npm run deploy`

### Netlify
1. Connect your Git repository to Netlify
2. Set build command to `npm run build`
3. Set publish directory to `build/`

### Vercel
1. Import your Git repository into Vercel
2. Set framework preset to "Docusaurus"
3. Set build command to `npm run build`
4. Set output directory to `build/`