---
title: Content Download and Offline Access
sidebar_position: 3
---

# Content Download and Offline Access

## Overview

The Physical AI & Humanoid Robotics textbook provides download capabilities that allow students to access content offline. This feature is particularly useful for students with limited internet access or those who prefer to study without connectivity.

## How It Works

Students can use the "Download for Offline Use" button on any module or weekly content page to download that content to their device. The content is provided in PDF format and includes all text content and diagrams.

## For Students

1. Navigate to the content you wish to download
2. Click the "Download for Offline Use" button
3. Save the file to your device
4. Access the content without an internet connection

## For Instructors

Instructors can generate downloads for entire modules to distribute to students who have limited internet access. Use the content generation scripts to prepare PDF versions of content in advance.

### Generating Downloads

To generate downloads for specific content:

```bash
npm run download:content [module] [week?option]
```

Examples:
- `npm run download:content ros2 week1-2` - Generate download for Week 1-2 of ROS 2 module
- `npm run download:content ros2` - Generate download for entire ROS 2 module
- `npm run download:content ros2 manifest` - Get list of available downloads

## Technical Implementation

The download functionality is implemented using:

1. A Node.js script (`scripts/download-content.js`) that can generate downloadable content
2. A React component (`DownloadButton.tsx`) that provides the user interface for downloading
3. A static file serving mechanism that makes downloads available at `/static/downloads/`

## Best Practices

- Encourage students to download content in advance when they have good connectivity
- Provide guidance on storage requirements (each module can be 50-100MB)
- Inform students about the update policy for downloaded content
- Consider time zones and access patterns when scheduling content updates

## Limitations

- Interactive elements may not work in downloaded content
- Some diagrams and code examples may have limited formatting in PDF
- Students will need to download new versions for updated content
- Large downloads may take time on slower connections

## Troubleshooting

**Q: The download button is grayed out**
A: Not all content is available for download yet. Check back later or contact your instructor.

**Q: The downloaded file doesn't open**
A: Try opening with a PDF viewer or contact support.

**Q: Can I download the entire textbook?**
A: Currently, downloads are available by module or week. Complete textbook downloads may be available in future releases.