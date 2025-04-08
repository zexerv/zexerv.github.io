/**
 * ==========================================================================
 * Blog Page Specific JavaScript (pages/blog.js)
 *
 * Dynamically loads all blog posts into the posts grid.
 * ==========================================================================
 */

document.addEventListener("DOMContentLoaded", function () {
    console.log("Blog Page JavaScript Initialized");
  
    const postsContainer = document.getElementById("posts");
  
    // Array of blog posts (same as in original script.js)
    // Consider moving this to a separate JSON file later for easier management
    const blogPosts = [
      {
        title: "The Journey Begins",
        content: "Welcome to my personal blog! This is where I'll share my thoughts, experiences, and insights.",
        date: "February 2, 2025", // Use consistent date format
        link: "blog/journey-begins.html"
      },
      {
        title: "Why Learning Every Day Matters",
        content: "Continuous learning is the key to growth. Whether it's reading, experimenting, or thinking critically, learning should never stop.",
        date: "February 8, 2025",
        link: "blog/learning-matters.html"
      },
      {
        title: "The Bellman Optimal Q-Value Function",
        content: "In Reinforcement Learning, the Q-value function evaluates the best action to take at a given state using the Bellman optimality equation.",
        date: "February 25, 2025",
        link: "blog/bellman-q-value.html"
      }
      // Add more posts here or load from JSON
    ];
  
    if (postsContainer) {
      // Clear any static placeholder content (if added previously)
      // postsContainer.innerHTML = ''; // Uncomment if placeholders were added to blog.html
  
      if (blogPosts.length > 0) {
        blogPosts.forEach((post) => {
          const article = document.createElement("article");
          article.classList.add("card", "post"); // Use card styles
  
          // Format content snippet (optional: truncate long content)
          let snippet = post.content;
          const maxLength = 150; // Max characters for snippet
          if (snippet.length > maxLength) {
            snippet = snippet.substring(0, maxLength) + "...";
          }
  
          article.innerHTML = `
            <div class="card-body">
              <h3 class="card-title">
                <a href="${post.link}" class="card-link">${post.title}</a>
              </h3>
              <small class="card-text text-muted"> Posted on ${post.date}
              </small>
              <p class="card-text">${snippet}</p>
              <a href="${post.link}" class="card-link">
                Read More <i class="fas fa-arrow-right"></i>
              </a>
            </div>
          `;
          postsContainer.appendChild(article);
        });
         console.log(`Loaded ${blogPosts.length} posts into #posts container.`);
      } else {
          postsContainer.innerHTML = '<p>No blog posts available yet.</p>';
           console.log("No blog posts found to load.");
      }
    } else {
      console.warn("Posts container (#posts) not found on this page.");
    }
  
  }); // End DOMContentLoaded