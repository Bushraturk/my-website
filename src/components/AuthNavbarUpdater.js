import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

// Function to update authentication links in the navbar
function updateAuthLinks() {
  if (!ExecutionEnvironment.canUseDOM) return;

  // Look for the element in the navbar
  const authLinksContainer = document.getElementById('auth-links-nav-element');
  if (!authLinksContainer) {
    // Retry after a short delay if the element is not yet available
    setTimeout(updateAuthLinks, 100);
    return;
  }

  // Clear previous content
  authLinksContainer.innerHTML = '';

  // Check authentication state from localStorage
  let isAuthenticated = false;
  let userName = '';

  if (typeof window !== 'undefined' && window.localStorage) {
    const mockUser = localStorage.getItem('mockUser');
    if (mockUser) {
      try {
        const userData = JSON.parse(mockUser);
        isAuthenticated = true;
        userName = userData.name || userData.user?.name || userData.user?.email || 'User';
      } catch (e) {
        console.error('Error parsing mockUser from localStorage:', e);
      }
    }
  }

  // Create and add auth links based on authentication status
  if (isAuthenticated) {
    // User is authenticated - show profile and logout
    authLinksContainer.innerHTML = `
      <div class="navbar__item dropdown dropdown--navbar dropdown--right dropdown--breakpoint">
        <a class="navbar__link navbar__item dropdown__link dropdown--right" href="javascript:void(0)" role="button" aria-haspopup="true">
          ${userName} ▾
        </a>
        <ul class="dropdown__menu">
          <li><a id="profile-link" class="dropdown__link">Profile</a></li>
          <li><button id="logout-btn" class="dropdown__link dropdown__link--btn" style="width:100%;text-align:left;background:none;border:none;cursor:pointer;">Logout</button></li>
        </ul>
      </div>
    `;

    // After injecting the HTML, add event listeners for navigation
    setTimeout(() => {
      // Add click handler for profile link
      const profileLink = document.getElementById('profile-link');
      if (profileLink) {
        profileLink.addEventListener('click', function(e) {
          e.preventDefault();
          window.location.href = window.location.origin + '/my-website/user-profile';
        });
      }

      // Also manually add click behavior for the dropdown since Docusaurus might not catch it
      const dropdownElement = authLinksContainer.querySelector('.dropdown');
      const dropdownToggle = authLinksContainer.querySelector('.navbar__link.dropdown__link');
      const dropdownMenu = authLinksContainer.querySelector('.dropdown__menu');

      if (dropdownElement && dropdownToggle && dropdownMenu) {
        // Ensure the dropdown element has proper toggle behavior
        dropdownToggle.addEventListener('click', function(e) {
          e.preventDefault();
          e.stopPropagation();

          // Toggle visibility by adding/removing CSS class
          dropdownElement.classList.toggle('dropdown--show');

          // Also toggle the menu display directly to ensure visibility
          if (dropdownMenu.style.display === 'block') {
            dropdownMenu.style.display = 'none';
          } else {
            dropdownMenu.style.display = 'block';
          }
        });

        // Close dropdown when clicking elsewhere
        document.addEventListener('click', function(event) {
          if (!dropdownElement.contains(event.target)) {
            dropdownElement.classList.remove('dropdown--show');
            dropdownMenu.style.display = 'none';
          }
        });
      }
    }, 300); // Slight delay to ensure DOM has updated

    // Add event listener to the logout button
    const logoutBtn = document.getElementById('logout-btn');
    if (logoutBtn) {
      logoutBtn.addEventListener('click', function() {
        if (typeof window !== 'undefined' && window.localStorage) {
          localStorage.removeItem('mockUser');
          localStorage.removeItem('activeSession');
          // Update UI immediately
          updateAuthLinks();
        }
      });
    }
  } else {
    // User is not authenticated - show sign in/up
    authLinksContainer.innerHTML = `
      <div class="navbar__items navbar__items--right">
        <a id="signin-link" class="button button--secondary button--outline navbar__link" style="margin-right:0.5rem;">Sign In</a>
        <a id="signup-link" class="button button--primary navbar__link">Sign Up</a>
      </div>
    `;

    // After injecting the HTML, add event listeners for navigation
    setTimeout(() => {
      // Add click handler for signin link
      const signinLink = document.getElementById('signin-link');
      if (signinLink) {
        signinLink.addEventListener('click', function(e) {
          e.preventDefault();
          window.location.href = window.location.origin + '/my-website/signin';
        });
      }

      // Add click handler for signup link
      const signupLink = document.getElementById('signup-link');
      if (signupLink) {
        signupLink.addEventListener('click', function(e) {
          e.preventDefault();
          window.location.href = window.location.origin + '/my-website/signup';
        });
      }
    }, 300); // Slight delay to ensure DOM has updated
  }
}

export default function AuthNavbarUpdater() {
  if (ExecutionEnvironment.canUseDOM) {
    // Update when page loads and whenever auth state changes
    updateAuthLinks();

    // Add storage event listener to respond to auth state changes
    const handleStorageChange = (e) => {
      if (e.key === 'mockUser' || e.key === 'activeSession') {
        updateAuthLinks();
      }
    };

    window.addEventListener('storage', handleStorageChange);

    // Clean up event listener
    return () => {
      window.removeEventListener('storage', handleStorageChange);
    };
  }

  return null; // Render nothing on the server side
}