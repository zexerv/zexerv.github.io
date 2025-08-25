// assets/js/pages/academia.js

document.addEventListener('DOMContentLoaded', () => {
    // Abstract Toggling
    const abstractToggles = document.querySelectorAll('.publication-abstract-toggle');

    abstractToggles.forEach(toggle => {
        toggle.addEventListener('click', (event) => {
            event.preventDefault();
            const abstractDiv = toggle.closest('.publication').querySelector('.publication-abstract');
            const icon = toggle.querySelector('i');

            if (abstractDiv) {
                abstractDiv.classList.toggle('active');
                toggle.classList.toggle('active'); // For styling the button itself if needed
                if (abstractDiv.classList.contains('active')) {
                    icon.classList.remove('fa-chevron-down');
                    icon.classList.add('fa-chevron-up');
                    toggle.setAttribute('aria-expanded', 'true');
                    abstractDiv.setAttribute('aria-hidden', 'false');
                } else {
                    icon.classList.remove('fa-chevron-up');
                    icon.classList.add('fa-chevron-down');
                     toggle.setAttribute('aria-expanded', 'false');
                     abstractDiv.setAttribute('aria-hidden', 'true');
                }
            }
        });

        // Set initial ARIA attributes
        const abstractDiv = toggle.closest('.publication').querySelector('.publication-abstract');
         if (abstractDiv) {
             toggle.setAttribute('aria-expanded', 'false');
             abstractDiv.setAttribute('aria-hidden', 'true');
             // Add controls attribute linking button to the abstract content
             const abstractId = 'abstract-' + Math.random().toString(36).substr(2, 9); // Generate unique ID
             abstractDiv.id = abstractId;
             toggle.setAttribute('aria-controls', abstractId);
         }
    });


    // Simple Formspree status handling (Optional, Formspree provides redirects)
    const contactForm = document.getElementById('contact-form-simplified');
    const formStatus = document.getElementById('form-status-simplified');

    async function handleSubmit(event) {
        event.preventDefault();
        const form = event.target;
        const data = new FormData(form);

        try {
            const response = await fetch(form.action, {
                method: form.method,
                body: data,
                headers: {
                    'Accept': 'application/json'
                }
            });

            if (response.ok) {
                if (formStatus) {
                    formStatus.textContent = "Thanks for your message! I'll get back to you soon.";
                    formStatus.className = 'form-status success'; // Use CSS classes
                }
                form.reset();
            } else {
                response.json().then(data => {
                    if (formStatus) {
                        if (Object.hasOwn(data, 'errors')) {
                             formStatus.textContent = data["errors"].map(error => error["message"]).join(", ");
                        } else {
                             formStatus.textContent = "Oops! There was a problem submitting your form.";
                        }
                         formStatus.className = 'form-status error'; // Use CSS classes
                    }
                })
            }
        } catch (error) {
             if (formStatus) {
                formStatus.textContent = "Oops! There was a problem submitting your form.";
                 formStatus.className = 'form-status error'; // Use CSS classes
             }
        } finally {
             // Re-enable submit button if disabled during submission
        }
    }

    if (contactForm) {
        contactForm.addEventListener("submit", handleSubmit);
    }
});