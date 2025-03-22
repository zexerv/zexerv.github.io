document.addEventListener("DOMContentLoaded", function () {
  console.log("JavaScript Loaded");

  // ----------------------------
  // Mobile Menu Toggle
  // ----------------------------
  const mobileMenuButton = document.getElementById("mobile-menu-btn");
  const navLinks = document.querySelector(".nav-links");
  
  if (mobileMenuButton) {
    mobileMenuButton.addEventListener("click", function() {
      navLinks.classList.toggle("active");
      mobileMenuButton.classList.toggle("open");
    });
  }
  
  // Close mobile menu when a link is clicked
  const menuLinks = document.querySelectorAll(".nav-links a");
  menuLinks.forEach(link => {
    link.addEventListener("click", function() {
      if (window.innerWidth <= 768) {
        navLinks.classList.remove("active");
        if (mobileMenuButton) {
          mobileMenuButton.classList.remove("open");
        }
      }
    });
  });

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
        link: "blog/journey-begins.html"
      },
      {
        title: "Why Learning Every Day Matters",
        content:
          "Continuous learning is the key to growth. Whether it's reading, experimenting, or thinking critically, learning should never stop.",
        date: "February 8, 2025",
        link: "blog/learning-matters.html"
      },
      {
        title: "The Bellman Optimal Q-Value Function",
        content:
          "In Reinforcement Learning, the Q-value function evaluates the best action to take at a given state using the Bellman optimality equation.",
        date: "February 25, 2025",
        link: "blog/bellman-q-value.html"
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

      // Submit the form (Formspree handles this now)
      const formData = new FormData(contactForm);
      fetch(contactForm.action, {
        method: 'POST',
        body: formData,
        headers: {
          'Accept': 'application/json'
        }
      })
      .then(response => {
        if (response.ok) {
          document.getElementById("form-status").innerHTML = `
            <div class="success-message">
              <i class="fas fa-check-circle"></i>
              Message sent successfully!
            </div>
          `;
          contactForm.reset();
        } else {
          document.getElementById("form-status").innerHTML = `
            <div class="error-message">
              <i class="fas fa-exclamation-circle"></i>
              Oops! There was a problem. Please try again.
            </div>
          `;
        }
        
        setTimeout(() => {
          document.getElementById("form-status").innerHTML = '';
        }, 5000);
      })
      .catch(error => {
        console.error('Error:', error);
      });
    });
  }
  
  // ----------------------------
  // Highlight Active Nav Link
  // ----------------------------
  const currentPage = window.location.pathname.split("/").pop();
  document.querySelectorAll(".nav-links a").forEach(link => {
    if (link.getAttribute("href") === currentPage) {
      link.classList.add("active");
    }
  });
});