/**
 * ==========================================================================
 * Academia Page Specific JavaScript (pages/academia.js)
 *
 * Handles the toggling of publication abstract visibility.
 * ==========================================================================
 */

document.addEventListener('DOMContentLoaded', function() {
    console.log("Academia Page JavaScript Initialized");
  
    // --- Publication Abstract Toggle ---
    const abstractToggles = document.querySelectorAll('.publication-abstract-toggle');
  
    if (abstractToggles.length > 0) {
      abstractToggles.forEach(toggle => {
        toggle.addEventListener('click', function(e) {
          e.preventDefault(); // Prevent link default action
  
          // Find the abstract element related to this toggle.
          // Assumes the abstract div immediately follows the paragraph containing the toggle link.
          const publicationLinksParagraph = this.closest('.publication-links'); // Find parent <p>
          const abstractElement = publicationLinksParagraph ? publicationLinksParagraph.nextElementSibling : null;
  
          if (abstractElement && abstractElement.classList.contains('publication-abstract')) {
            // Toggle the .show class on the abstract
            abstractElement.classList.toggle('show');
  
            // Toggle the icon class on the link
            const icon = this.querySelector('i'); // Find the icon within the link
            if (icon) {
              const isShown = abstractElement.classList.contains('show');
              icon.classList.toggle('fa-chevron-down', !isShown);
              icon.classList.toggle('fa-chevron-up', isShown);
            }
          } else {
              console.warn("Could not find the corresponding abstract element for toggle:", this);
          }
        });
      });
       console.log("Abstract toggle listeners added.");
    } else {
        console.log("No publication abstract toggles found on this page.");
    }
  
  }); // End DOMContentLoaded