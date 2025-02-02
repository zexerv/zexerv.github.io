document.addEventListener("DOMContentLoaded", function () {
    console.log("JavaScript Loaded");
  
    // ----------------------------
    // Dynamic Blog Posts Loading
    // ----------------------------
    const postsContainer = document.getElementById("posts");
    if (postsContainer) {
      // Array of blog posts to be rendered dynamically
      const blogPosts = [
        {
          title: "The Journey Begins",
          content:
            "Welcome to my personal blog! This is where I'll share my thoughts, experiences, and insights.",
          date: "February 2, 2025",
          link: "assets/posts/journey-begins.html"
        },
        {
          title: "Why Learning Every Day Matters",
          content:
            "Continuous learning is the key to growth. Whether it's reading, experimenting, or thinking critically, learning should never stop.",
          date: "February 8, 2025",
          link: "assets/posts/learning-matters.html"
        },
        {
          title: "The Bellman Optimal Q-Value Function",
          content:
            "In Reinforcement Learning, the Q-value function evaluates the best action to take at a given state using the Bellman optimality equation.",
          date: "February 25, 2025",
          link: "assets/posts/bellman-q-value.html"
        }
      ];
  
      blogPosts.forEach((post) => {
        const article = document.createElement("article");
        article.classList.add("post");
        article.innerHTML = `
          <h2><a href="${post.link}">${post.title}</a></h2>
          <p>${post.content}</p>
          <small>Posted on ${post.date}</small>
        `;
        postsContainer.appendChild(article);
      });
    }
  
    // ----------------------------
    // Contact Form Handling
    // ----------------------------
    const contactForm = document.getElementById("contact-form");
    if (contactForm) {
      contactForm.addEventListener("submit", function (event) {
        event.preventDefault();
  
        // Basic form validation
        const name = document.getElementById("name").value.trim();
        const email = document.getElementById("email").value.trim();
        const message = document.getElementById("message").value.trim();
  
        if (!name || !email || !message) {
          alert("Please fill in all the fields.");
          return;
        }
  
        // Simulate sending a message (this could be replaced with an AJAX call)
        alert("Thank you for your message! I'll get back to you soon.");
        contactForm.reset();
      });
    }
  });
  