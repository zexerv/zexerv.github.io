/**
 * ==========================================================================
 * Contact Page Specific JavaScript (pages/contact.js)
 *
 * Handles the contact form submission using Fetch API to Formspree.
 * ==========================================================================
 */

document.addEventListener("DOMContentLoaded", function () {
    console.log("Contact Page JavaScript Initialized");
  
    const form = document.getElementById('contact-form');
    const formStatus = document.getElementById('form-status');
    const submitBtn = form ? form.querySelector('button[type="submit"]') : null;
  
    if (form && formStatus && submitBtn) {
      form.addEventListener('submit', async (e) => {
        e.preventDefault(); // Prevent default browser submission
  
        // --- Form Validation (Basic) ---
        const name = form.elements['name'].value.trim();
        const email = form.elements['email'].value.trim();
        const subject = form.elements['subject'].value.trim();
        const message = form.elements['message'].value.trim();
  
        if (!name || !email || !subject || !message) {
          formStatus.innerHTML = `
            <div class="error-message">
              <i class="fas fa-exclamation-circle"></i>
              Please fill out all fields before sending.
            </div>`;
          // Auto-hide validation message
          setTimeout(() => { formStatus.innerHTML = ''; }, 5000);
          return; // Stop submission if fields are empty
        }
        // --- End Validation ---
  
  
        // --- Handle Submission ---
        const originalBtnHTML = submitBtn.innerHTML;
        submitBtn.innerHTML = '<i class="fas fa-spinner fa-spin"></i> Sending...';
        submitBtn.disabled = true;
        formStatus.innerHTML = ''; // Clear previous status messages
  
        try {
          const formData = new FormData(form);
          const response = await fetch(form.action, {
            method: 'POST',
            body: formData,
            headers: {
              'Accept': 'application/json' // Required by Formspree for AJAX
            }
          });
  
          if (response.ok) {
            // Success
            form.reset(); // Clear the form fields
            formStatus.innerHTML = `
              <div class="success-message">
                <i class="fas fa-check-circle"></i>
                Thank you! Your message has been sent successfully.
              </div>`;
          } else {
            // Handle server-side errors from Formspree (e.g., validation)
            const responseData = await response.json();
            let errorMessage = 'Oops! There was a problem sending your message.';
            if (responseData && responseData.errors) {
                errorMessage = responseData.errors.map(error => error.message).join('<br>');
            }
            formStatus.innerHTML = `
              <div class="error-message">
                <i class="fas fa-exclamation-circle"></i>
                ${errorMessage} Please try again.
              </div>`;
          }
        } catch (error) {
          // Handle network errors
          console.error('Form submission error:', error);
          formStatus.innerHTML = `
            <div class="error-message">
              <i class="fas fa-exclamation-circle"></i>
              Network error. Please check your connection and try again.
            </div>`;
        } finally {
          // Restore button state regardless of success/failure
          submitBtn.innerHTML = originalBtnHTML;
          submitBtn.disabled = false;
  
          // Optional: Auto-hide success/error message after a delay
          setTimeout(() => {
            if (formStatus.querySelector('.success-message, .error-message')) {
              formStatus.innerHTML = '';
            }
          }, 7000); // Hide after 7 seconds
        }
        // --- End Handle Submission ---
  
      }); // End form event listener
  
    } else {
      if (!form) console.warn("Contact form (#contact-form) not found.");
      if (!formStatus) console.warn("Form status element (#form-status) not found.");
      if (!submitBtn) console.warn("Form submit button not found.");
    }
  
  }); // End DOMContentLoaded