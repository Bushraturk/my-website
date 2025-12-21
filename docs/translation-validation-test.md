# Validation Test for Language Translation Feature

## Test Steps

1. **Initial Setup**
   - Start the Docusaurus development server
   - Navigate to the textbook website
   - Verify that the default language is English

2. **Language Toggle Test**
   - Click on the language toggle button in the navbar (appears as language selector)
   - Verify that the site loads the Urdu version of the page
   - Check that the page content appears in Urdu
   - Verify that navigation links and other UI elements are translated appropriately

3. **Content Verification**
   - Navigate to the ROS 2 module and verify content is in Urdu
   - Navigate to the Gazebo/Unity module and verify content is in Urdu
   - Navigate to the NVIDIA Isaac module and verify content is in Urdu
   - Navigate to the VLA module and verify content is in Urdu
   - Check that all content appears in the selected language

4. **Persistence Test**
   - Navigate to a different page while in Urdu
   - Verify that the language remains Urdu on all pages
   - Navigate to a different module within the textbook
   - Verify that language preference persists across navigation

5. **Toggle Back Test**
   - Click on the language toggle button to switch back to English
   - Verify that the site loads the English version of the page
   - Check that the page content appears in English
   - Verify that navigation links and other UI elements are in English

6. **Cross-Module Consistency Test**
   - Start in one module (e.g. ROS 2) and switch to Urdu
   - Navigate to a different module (e.g. Gazebo/Unity)
   - Verify that the language remains Urdu throughout navigation
   - Switch back to English using the language selector
   - Navigate back to the first module
   - Verify that the language remains English

7. **Specific Content Verification**
   - Navigate to the 'Introduction' page and verify translation
   - Navigate to 'Week 1-2' in ROS 2 module and verify translation
   - Navigate to 'Week 3' in ROS 2 module and verify translation
   - Repeat for Gazebo/Unity, NVIDIA Isaac, and VLA modules
   - Verify that both simple text and complex content are properly provided in selected language

## Expected Results

- The language toggle button should switch between available languages
- Each language loads a separate, fully translated static version of the site
- Language preference is maintained during navigation but handled as separate site versions
- Navigation between modules should show content in the selected language
- All text content should be properly provided according to translation in respective locale directories
- Code blocks and technical elements may remain in English depending on translation completeness
- No broken elements or untranslated content should appear when switching languages

## Known Issues

- May require initial load of each language version to see properly translated content
- Some content may not be fully translated if translation files are incomplete
- Switching languages involves loading a different static version of the site
- Requires proper translation files to exist in the i18n/ur directory structure
- Navigation after language switch may take slightly longer due to loading new static version