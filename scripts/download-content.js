/**
 * Content download utility for the Physical AI & Humanoid Robotics Textbook
 * This utility allows users to download content for offline use.
 */

const fs = require('fs');
const path = require('path');
const { exec } = require('child_process');
const util = require('util');

const execPromise = util.promisify(exec);

/**
 * Generate downloadable content for a given module and week
 * @param {string} module - The module name (e.g., 'ros2', 'gazebo-unity')
 * @param {string} week - The week identifier (e.g., 'week1-2', 'week3')
 * @returns {Promise<string>} Path to the generated download file
 */
async function generateDownloadableContent(module, week) {
  // Validate inputs
  if (!module || !week) {
    throw new Error('Module and week parameters are required');
  }

  // Define the source path for the content
  const contentPath = path.join(__dirname, '..', 'docs', module, `${week}.md`);
  
  // Check if the content file exists
  if (!fs.existsSync(contentPath)) {
    throw new Error(`Content file not found: ${contentPath}`);
  }

  // Create downloads directory if it doesn't exist
  const downloadsDir = path.join(__dirname, '..', 'static', 'downloads');
  if (!fs.existsSync(downloadsDir)) {
    fs.mkdirSync(downloadsDir, { recursive: true });
  }

  // Define the output path for the downloadable content
  const outputPath = path.join(downloadsDir, `${module}-${week}.pdf`);
  
  // For this implementation, we'll create a simple PDF using Puppeteer
  // In a real implementation, this would use a PDF generation library
  // or a service like Puppeteer to convert the markdown to PDF
  console.log(`Generating downloadable content for ${module}/${week}`);
  
  // In this example, we'll simply copy the markdown file as a temporary measure
  // A complete implementation would convert to a proper format like PDF
  const markdownContent = fs.readFileSync(contentPath, 'utf8');
  
  // Add title and metadata to the downloadable content
  const downloadableContent = `# ${module} - ${week}\n\n${markdownContent}`;
  
  // Write the downloadable content to the output file
  fs.writeFileSync(outputPath, downloadableContent);
  
  console.log(`Generated downloadable content at: ${outputPath}`);
  
  return outputPath;
}

/**
 * Generate downloadable content for a complete module
 * @param {string} module - The module name (e.g., 'ros2', 'gazebo-unity')
 * @returns {Promise<string>} Path to the generated download file
 */
async function generateModuleDownload(module) {
  if (!module) {
    throw new Error('Module parameter is required');
  }

  // Generate individual week downloads and then combine them
  // For this example, I'll just create a placeholder
  const downloadsDir = path.join(__dirname, '..', 'static', 'downloads');
  if (!fs.existsSync(downloadsDir)) {
    fs.mkdirSync(downloadsDir, { recursive: true });
  }

  const outputPath = path.join(downloadsDir, `${module}-complete.pdf`);
  
  // In a real implementation, this would combine all week files for the module
  fs.writeFileSync(outputPath, `Complete ${module} module content for offline use.`);
  
  console.log(`Generated module download at: ${outputPath}`);
  
  return outputPath;
}

/**
 * Get download manifest for a module
 * @param {string} module - The module name
 * @returns {object} Manifest with available downloads
 */
function getDownloadManifest(module) {
  const downloadsDir = path.join(__dirname, '..', 'static', 'downloads');
  
  if (!fs.existsSync(downloadsDir)) {
    return { module, availableDownloads: [] };
  }

  const files = fs.readdirSync(downloadsDir);
  const relevantFiles = files.filter(file => 
    file.startsWith(module) && (file.endsWith('.pdf') || file.endsWith('.zip'))
  );

  return {
    module,
    availableDownloads: relevantFiles.map(file => ({
      filename: file,
      url: `/downloads/${file}`,
      size: fs.statSync(path.join(downloadsDir, file)).size,
      lastModified: fs.statSync(path.join(downloadsDir, file)).mtime
    }))
  };
}

// CLI interface
if (require.main === module) {
  const args = process.argv.slice(2);
  
  if (args.length < 1) {
    console.log('Usage: node download-content.js [module] [week?option]');
    console.log('Example: node download-content.js ros2 week1-2  (Generate download for specific week)');
    console.log('Example: node download-content.js ros2           (Generate download for whole module)');
    console.log('Example: node download-content.js ros2 manifest  (Get download manifest)');
    process.exit(1);
  }

  const [module, command] = args;
  
  (async () => {
    try {
      if (command === 'manifest') {
        const manifest = getDownloadManifest(module);
        console.log(JSON.stringify(manifest, null, 2));
      } else if (command) {
        const result = await generateDownloadableContent(module, command);
        console.log(`Successfully generated: ${result}`);
      } else {
        const result = await generateModuleDownload(module);
        console.log(`Successfully generated: ${result}`);
      }
    } catch (error) {
      console.error(`Error: ${error.message}`);
      process.exit(1);
    }
  })();
}

module.exports = {
  generateDownloadableContent,
  generateModuleDownload,
  getDownloadManifest
};