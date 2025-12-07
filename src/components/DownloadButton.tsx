import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './DownloadButton.module.css';

const DownloadButton = ({ module, week, title = "Download for Offline Use" }) => {
  const [isAvailable, setIsAvailable] = useState(false);
  const [downloadUrl, setDownloadUrl] = useState('');
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    // Check if the download is available
    if (module && (week || !module.includes('/'))) {
      // In a real implementation, this would check against a manifest API
      // For now, we'll construct the URL based on module and week
      let fileName;
      if (week) {
        fileName = `${module}-${week}.pdf`;
      } else {
        fileName = `${module}-complete.pdf`;
      }
      
      // This would be an actual check to see if the file exists
      // For now, we'll assume it's available for the ROS 2 module as an example
      if (module === 'ros2') {
        setIsAvailable(true);
        setDownloadUrl(`/downloads/${fileName}`);
      } else {
        setIsAvailable(false);
      }
    }
  }, [module, week]);

  const handleDownload = async () => {
    if (!isAvailable || !downloadUrl) return;

    setLoading(true);
    
    try {
      // In a real implementation, this would track the download
      // and potentially increment download counts
      console.log(`Downloading: ${downloadUrl}`);
      
      // Actual download happens via the browser when the link is clicked
    } catch (error) {
      console.error('Download failed:', error);
    } finally {
      setLoading(false);
    }
  };

  if (!isAvailable) {
    return (
      <div className={styles.downloadContainer}>
        <button 
          className={clsx('button button--secondary button--lg', styles.downloadButton, styles.unavailable)}
          disabled={true}
        >
          {title} (Coming Soon)
        </button>
        <p className={styles.infoText}>
          This content is not yet available for offline download, but will be soon.
        </p>
      </div>
    );
  }

  return (
    <div className={styles.downloadContainer}>
      <a 
        href={downloadUrl} 
        className={clsx('button button--primary button--lg', styles.downloadButton)}
        onClick={handleDownload}
        download
      >
        {loading ? (
          <span className={styles.loadingSpinner}>⏳</span>
        ) : (
          <span className={styles.downloadIcon}>📥</span>
        )}
        {title}
      </a>
      <p className={styles.infoText}>
        Download this content to your device for offline access.
      </p>
    </div>
  );
};

export default DownloadButton;