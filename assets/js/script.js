document.addEventListener("DOMContentLoaded", function() {
    console.log("JavaScript Loaded");

    const postsSection = document.getElementById("posts");

    // Function to dynamically add a blog post
    function addPost(title, content, date) {
        const article = document.createElement("article");
        article.innerHTML = `
            <h2>${title}</h2>
            <p>${content}</p>
            <small>Posted on ${date}</small>
        `;
        postsSection.appendChild(article);
    }

    // Adding a new post dynamically
    addPost("How I Stay Motivated in Research", "Keeping motivation alive while working on long-term projects can be difficult, but I've found some strategies that help.", "February 20, 2025");

    // Contact Form Submission
    const contactForm = document.getElementById("contact-form");
    if (contactForm) {
        contactForm.addEventListener("submit", function(event) {
            event.preventDefault();
            alert("Thank you for your message! I'll get back to you soon.");
            contactForm.reset();
        });
    }
});
