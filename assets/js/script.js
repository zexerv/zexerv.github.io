document.addEventListener("DOMContentLoaded", function() {
    console.log("JavaScript Loaded");
  
    // Responsive Navigation Toggle
    const menuToggle = document.querySelector('.menu-toggle');
    const navLinks = document.querySelector('.nav-links');
  
    if (menuToggle) {
      menuToggle.addEventListener("click", () => {
        navLinks.classList.toggle("nav-active");
      });
    }
  
    // Blog posts data (three sample posts)
    const blogPosts = [
      {
        title: "The Journey Begins",
        content: "Welcome to my personal blog! This is where I'll share my thoughts, experiences, and insights.",
        date: "February 2, 2025"
      },
      {
        title: "Why Learning Every Day Matters",
        content: "Continuous learning is the key to growth. Whether it's reading, experimenting, or thinking critically about the world, learning should never stop.",
        date: "February 8, 2025"
      },
      {
        title: "The Bellman Optimal Q-Value Function",
        content: `
          In Reinforcement Learning, the <strong>Q-value function</strong> evaluates the best action to take at a given state. The Bellman optimality equation defines this recursively:
          <br><br>
          <pre>Q*(s, a) = E [r + γ max(Q*(s', a')) | s, a]</pre>
          <br>
          Where:
          <ul>
            <li>\( s, a \) → Current state and action.</li>
            <li>\( s', a' \) → Next state and possible next action.</li>
            <li>\( r \) → Immediate reward.</li>
            <li>\( \gamma \) → Discount factor for future rewards.</li>
            <li>\( \max_{a'} Q^*(s', a') \) → The highest expected reward for the next state.</li>
          </ul>
          This equation is fundamental in Q-learning and Deep Q Networks (DQNs), allowing an agent to learn the best policy over time.
        `,
        date: "February 25, 2025"
      }
    ];
  
    // Function to add posts dynamically if #posts element exists
    const postsSection = document.getElementById("posts");
    if (postsSection) {
      // Clear existing content if any
      postsSection.innerHTML = "";
      blogPosts.forEach(post => {
        const article = document.createElement("article");
        article.classList.add("post-item");
        article.innerHTML = `
          <h2>${post.title}</h2>
          <p>${post.content}</p>
          <small>Posted on ${post.date}</small>
        `;
        postsSection.appendChild(article);
      });
    }
  
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
  