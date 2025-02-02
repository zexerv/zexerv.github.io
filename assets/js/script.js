document.addEventListener("DOMContentLoaded", function() {
    console.log("JavaScript Loaded");

    const postsSection = document.getElementById("posts");

    // Example of adding a new blog post dynamically
    function addPost(title, content, date) {
        const article = document.createElement("article");
        article.innerHTML = `
            <h2>${title}</h2>
            <p>${content}</p>
            <small>Posted on ${date}</small>
        `;
        postsSection.appendChild(article);
    }

    // Adding a sample post dynamically
    addPost("New Blog Update!", "I'm excited to share my new thoughts soon.", "February 10, 2025");
});
