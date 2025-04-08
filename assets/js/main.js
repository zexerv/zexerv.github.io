/**
 * ==========================================================================
 * Main JavaScript File
 *
 * Contains global scripts needed across the site, like the mobile menu toggle.
 * ==========================================================================
 */

document.addEventListener("DOMContentLoaded", function () {
    console.log("Main JavaScript Initialized");
  
    // --- Mobile Menu Toggle ---
    const mobileMenuButton = document.getElementById("mobile-menu-btn");
    const navLinks = document.querySelector(".nav-links");
  
    if (mobileMenuButton && navLinks) {
      mobileMenuButton.addEventListener("click", function () {
        navLinks.classList.toggle("active"); // Toggles visibility of the nav links container
        mobileMenuButton.classList.toggle("open"); // Toggles the button's appearance (e.g., hamburger to X)
      });
  
      // Optional: Close mobile menu when a link inside it is clicked
      const menuLinks = navLinks.querySelectorAll("a");
      menuLinks.forEach(link => {
        link.addEventListener("click", function () {
          // Check if the mobile menu is active before closing
          if (navLinks.classList.contains("active")) {
            navLinks.classList.remove("active");
            if (mobileMenuButton) {
              mobileMenuButton.classList.remove("open");
            }
          }
        });
      });
  
      // Optional: Close mobile menu if user clicks outside of it
      document.addEventListener('click', function(event) {
        const isClickInsideNav = navLinks.contains(event.target);
        const isClickOnButton = mobileMenuButton.contains(event.target);
  
        if (!isClickInsideNav && !isClickOnButton && navLinks.classList.contains('active')) {
          navLinks.classList.remove("active");
          if (mobileMenuButton) {
             mobileMenuButton.classList.remove("open");
          }
        }
      });
  
    } else {
      console.warn("Mobile menu button or nav links element not found.");
    }
  
    // --- Add other global scripts here if needed in the future ---
    // Example: Smooth scroll for anchor links, theme toggle, etc.
  
  }); // End DOMContentLoaded