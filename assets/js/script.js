document.addEventListener("DOMContentLoaded", function() {
    console.log("JavaScript Loaded");

    const postsSection = document.getElementById("posts");

    function addPost(title, content, date) {
        const article = document.createElement("article");
        article.innerHTML = `
            <h2>${title}</h2>
            <p>${content}</p>
            <small>Posted on ${date}</small>
        `;
        postsSection.appendChild(article);
    }

    // Adding a blog post dynamically
    addPost(
        "Understanding the Bellman Q-Value Function",
        "In reinforcement learning, the Q-function helps determine the best possible action at a given state using the Bellman optimality equation.",
        "February 25, 2025"
    );

    // Contact Form Handling
    const contactForm = document.getElementById("contact-form");
    if (contactForm) {
        contactForm.addEventListener("submit", function(event) {
            event.preventDefault();
            alert("Thank you for your message! I'll get back to you soon.");
            contactForm.reset();
        });
    }
});
