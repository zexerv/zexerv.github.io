/**
 * ==========================================================================
 * Resume Page Specific JavaScript (pages/resume.js)
 *
 * Initializes Tippy.js tooltips for paper previews and handles
 * skill bar animations on scroll.
 * ==========================================================================
 */

document.addEventListener("DOMContentLoaded", function () {
    console.log("Resume Page JavaScript Initialized");
  
    // --- Tippy.js Tooltip Initialization ---
    // Ensure Tippy library is loaded (from CDN in HTML) before running
    if (typeof tippy === 'function') {
      tippy('[data-tippy-content]', {
        allowHTML: true,         // Allow HTML content from the attribute
        placement: 'right',        // Default placement
        animation: 'scale',        // Animation style
        theme: 'light',          // Use a light theme (or create a custom one)
        maxWidth: 250,           // Max width of the tooltip
        interactive: true,         // Allow interaction (hovering over tooltip)
        arrow: true,             // Show arrow pointing to the element
        // Use the custom class defined in resume.css for styling overrides
        // Note: theme:'light' might conflict if .paper-tooltip defines background etc.
        // Adjust theme or CSS (.tippy-box[data-theme~='light']) as needed.
        // Using 'light-border' theme which is often good for images:
        // theme: 'light-border', // Or remove theme and rely solely on CSS
        popperOptions: {
           modifiers: [ { name: 'preventOverflow', options: { padding: 10 } } ] // Add padding from viewport edges
        },
        // Ensure high z-index if needed (usually handled by tippy itself or CSS)
        // zIndex: 9999,
        // Append to body to avoid parent overflow issues
        appendTo: () => document.body,
      });
       console.log("Tippy.js initialized for paper previews.");
    } else {
      console.warn("Tippy.js library not found. Tooltips will not work.");
    }
  
  
    // --- Skill Bar Animation on Scroll ---
    const skillsSection = document.querySelector('.skills-container'); // Target the container
    let skillsAnimated = false; // Flag to run animation only once
  
    // Helper function to check if an element is (partially) in the viewport
    function isElementInViewport(el) {
      if (!el) return false;
      const rect = el.getBoundingClientRect();
      return (
        rect.top < (window.innerHeight || document.documentElement.clientHeight) &&
        rect.bottom > 0 && // Ensure bottom is below the top of the viewport
        rect.left < (window.innerWidth || document.documentElement.clientWidth) &&
        rect.right > 0 // Ensure right is past the left of the viewport
      );
    }
  
    // Function to animate the skill bars
    function animateSkills() {
      const skillLevelDivs = document.querySelectorAll('.skill-level div');
      skillLevelDivs.forEach((skillDiv) => {
        const level = skillDiv.style.getPropertyValue('--level');
        if (level) {
          // Directly setting width; transition is handled by CSS
          skillDiv.style.width = level;
        }
      });
       console.log("Skill bar animation triggered.");
    }
  
    // Function to handle scroll event
    function handleScroll() {
      if (!skillsAnimated && isElementInViewport(skillsSection)) {
        animateSkills();
        skillsAnimated = true; // Set flag so it doesn't run again
        // Optional: Remove listener after animation runs
        // window.removeEventListener('scroll', handleScroll);
        // console.log("Scroll listener for skills removed.");
      }
    }
  
    // Add scroll event listener if skills section exists
    if (skillsSection) {
      // Initial check in case the section is already visible on load
      handleScroll();
      // Add listener for scrolling
      window.addEventListener('scroll', handleScroll);
    } else {
       console.warn("Skills container (.skills-container) not found for animation.");
    }
  
  
  }); // End DOMContentLoaded