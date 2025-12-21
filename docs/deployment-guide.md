---
title: Deployment Guide
sidebar_position: 30
---

# Deployment Guide

This guide provides comprehensive instructions for deploying the Physical AI & Humanoid Robotics textbook website. It covers local development setup, CI/CD processes, and production deployment strategies.

## Prerequisites

### System Requirements
- **Operating System**: Linux, macOS, or Windows with WSL2
- **Node.js**: Version 18.x or higher
- **npm**: Version 8.x or higher (or yarn)
- **Git**: Version control system
- **Disk Space**: At least 1GB for dependencies

### Development Dependencies
```bash
node --version  # Should be >= 18.x
npm --version   # Should be >= 8.x
git --version   # Should be >= 2.x
```

## Local Development Setup

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/physical-ai-textbook.git
cd physical-ai-textbook
```

### 2. Install Dependencies
```bash
npm install
# or
yarn install
```

### 3. Start Development Server
```bash
npm start
# or
yarn start
```

This will start the development server at `http://localhost:3000` with hot reloading enabled.

## Building for Production

### Basic Build
```bash
npm run build
# or
yarn build
```

This creates a `build` directory with static files optimized for deployment.

### Production Build with Analytics
```bash
npm run build -- --bundle-analyzer
# or
yarn build --bundle-analyzer
```

This provides insights into bundle size and possible optimizations.

## Deployment Options

### 1. GitHub Pages (Default)

#### Prerequisites
- GitHub repository with GitHub Pages enabled
- Write access to the repository

#### Configuration
Update `docusaurus.config.ts`:
```typescript
const config = {
  // ...
  url: 'https://your-org.github.io',
  baseUrl: '/my-website/',
  organizationName: 'your-org',
  projectName: 'my-website',
  // ...
};
```

#### Deployment Command
```bash
npm run deploy
# or
yarn deploy
```

This will build the site and push the static files to the `gh-pages` branch.

### 2. Vercel

#### Configuration File
Create `vercel.json` in project root:
```json
{
  "builds": [
    {
      "src": "package.json",
      "use": "@vercel/static-build",
      "config": {
        "distDir": "build"
      }
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "/index.html"
    }
  ]
}
```

#### Deployment
1. Install Vercel CLI: `npm i -g vercel`
2. Link project: `vercel`
3. Deploy: `vercel --prod`

### 3. Netlify

#### Configuration File
Create `_redirects` file in `static` folder:
```
/*    /index.html   200
```

#### Deployment
1. Connect your GitHub repository to Netlify
2. Set build command to: `npm run build`
3. Set publish directory to: `build`

### 4. Self-Hosted

#### Using Nginx
```nginx
server {
    listen 80;
    server_name your-domain.com;
    root /path/to/your/build/folder;
    index index.html;

    # Handle client-side routing
    location / {
        try_files $uri $uri/ /index.html;
    }

    # Gzip compression
    gzip on;
    gzip_types
        text/plain
        text/css
        text/xml
        text/javascript
        application/javascript
        application/xml+rss
        application/json;
}
```

#### Using Apache
```apache
# .htaccess file in build directory
Options -MultiViews
RewriteEngine On

# Handle client-side routing
RewriteCond %{REQUEST_FILENAME} !-d
RewriteCond %{REQUEST_FILENAME} !-f
RewriteRule ^ index.html [L]
```

## Environment Configuration

### Production Environment Variables
Create `.env.production`:
```
# API endpoints for production
API_BASE_URL=https://api.your-domain.com

# Analytics IDs
GA_TRACKING_ID=G-XXXXXXXXXX
MATOMO_URL=https://matomo.your-domain.com
MATOMO_SITE_ID=1

# Authentication settings
AUTH_SERVER_URL=https://auth.your-domain.com
```

### Development Environment Variables
Create `.env.development`:
```
# Local API endpoints
API_BASE_URL=http://localhost:8000

# Analytics (usually disabled in development)
GA_TRACKING_ID=
MATOMO_URL=
MATOMO_SITE_ID=
```

## Continuous Integration/Deployment (CI/CD)

### GitHub Actions Example

Create `.github/workflows/deploy.yml`:
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  test-deploy:
    if: github.event_name == 'pull_request'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci

      - name: Build website
        run: npm run build

  deploy:
    if: github.event_name == 'push' && github.ref == 'refs/heads/main'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci

      - name: Build website
        run: npm run build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
          user_name: github-actions[bot]
          user_email: 41898282+github-actions[bot]@users.noreply.github.com
```

## Performance Optimization for Deployment

### Bundle Size Reduction
1. **Code Splitting**: Docusaurus automatically handles this
2. **Image Optimization**: Use appropriate formats and sizes
3. **Dependency Optimization**: Audit and remove unused packages

### Build Optimization
```bash
# Analyze bundle size
npm run build -- --bundle-analyzer

# Check for performance issues
npm run serve
npx lighthouse http://localhost:3000 --view
```

## Monitoring and Analytics

### Google Analytics Setup
Update `docusaurus.config.ts`:
```typescript
module.exports = {
  // ...
  presets: [
    [
      'classic',
      {
        // ...
        gtag: {
          trackingID: 'G-XXXXXXXXXX',
          anonymizeIP: true,
        },
        // ...
      },
    ],
  ],
};
```

### Performance Monitoring
Set up performance monitoring with:
- Google Analytics custom events
- Web Vitals tracking
- Error reporting

## Security Considerations

### HTTPS Requirements
- Always deploy with HTTPS
- Use HSTS headers
- Enable secure cookies for authentication

### Content Security Policy
```html
<meta http-equiv="Content-Security-Policy" 
      content="default-src 'self'; 
               script-src 'self' 'unsafe-inline' 'unsafe-eval' cdn.jsdelivr.net;
               style-src 'self' 'unsafe-inline' cdn.jsdelivr.net;
               img-src 'self' data: cdn.jsdelivr.net;
               font-src 'self' cdn.jsdelivr.net;">
```

## Troubleshooting Deployment Issues

### Common Build Errors
1. **Out of Memory Error**:
   - Increase Node.js memory: `node --max-old-space-size=4096 node_modules/.bin/docusaurus build`
   - Check for circular dependencies in code

2. **Bundle Size Warnings**:
   - Review dependencies with `npm run build -- --bundle-analyzer`
   - Implement code splitting for large components
   - Optimize images and assets

3. **Broken Links**:
   - Run link validation: `npm run validate:content`
   - Check all internal and external links
   - Verify proper asset paths

### Debugging Production Issues
1. **Enable Source Maps**: Set `generateBuildArtifacts: true` in config
2. **Monitor Console**: Check browser console for errors
3. **Network Analysis**: Verify all assets load correctly
4. **Performance Analysis**: Check for slow-loading resources

## Backup and Recovery

### Content Backup
- All content is stored in Git repository
- Regular pushes to remote repository
- Tag releases for easy rollback

### Deployment Backup
- Maintain staging environment as backup
- Document rollback process
- Keep previous versions accessible

## Maintenance and Updates

### Regular Maintenance Tasks
1. **Dependency Updates**:
   ```bash
   npm outdated
   npm update
   ```

2. **Security Scans**:
   ```bash
   npm audit
   npm audit fix
   ```

3. **Performance Monitoring**:
   - Monthly Lighthouse audits
   - Daily uptime monitoring
   - Weekly performance trend analysis

### Rollback Procedures
1. **Staging Rollback**:
   - Revert to previous commit in staging
   - Test functionality
   - Deploy to production if valid

2. **Production Rollback**:
   - Use previous deployment artifacts
   - Revert Git repository to previous working state
   - Update DNS/CDN if necessary

## Testing in Production

### Pre-Deployment Testing
- Unit tests pass (coverage >85%)
- Integration tests pass
- Accessibility tests pass (axe-core)
- Performance tests pass (Core Web Vitals)

### Post-Deployment Testing
- URL accessibility check
- Content rendering verification
- Performance monitoring setup
- Error tracking activation

## Scaling Considerations

### Traffic Scaling
- CDN setup for static assets
- Caching strategies for dynamic content
- Monitoring for traffic spikes

### Content Scaling
- Efficient search implementation
- Optimized content loading
- Database scaling for user data

This deployment guide ensures that the Physical AI & Humanoid Robotics textbook website can be deployed reliably and maintained effectively across different environments and hosting providers.