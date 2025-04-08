/**
 * ==========================================================================
 * Blog Page Specific JavaScript (pages/blog.js) - UPDATED TO FETCH JSON
 *
 * Dynamically loads all blog posts from a JSON file into the posts grid.
 * ==========================================================================
 */

document.addEventListener("DOMContentLoaded", async function () { // Add async
    console.log("Blog Page JavaScript Initialized");
  
    const postsContainer = document.getElementById("posts");
    const dataUrl = '../../assets/data/blogPosts.json'; // Path relative to blog.js
  
    if (postsContainer) {
      postsContainer.innerHTML = '<p>Loading posts...</p>'; // Loading indicator
  
      try {
        const response = await fetch(dataUrl);
        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }
        const blogPosts = await response.json(); // Parse the JSON data
  
        postsContainer.innerHTML = ''; // Clear loading indicator
  
        if (blogPosts && blogPosts.length > 0) {
          // Assuming newest posts are first in the JSON file
          blogPosts.forEach((post) => {
            const article = document.createElement("article");
            article.classList.add("card", "post"); // Use card styles
  
            let snippet = post.content;
            const maxLength = 150;
            if (snippet.length > maxLength) {
              snippet = snippet.substring(0, maxLength) + "...";
            }
  
            article.innerHTML = `
              <div class="card-body">
                <h3 class="card-title">
                  <a href="${post.link}" class="card-link">${post.title}</a>
                </h3>
                <small class="card-text text-muted">
                  Posted on ${post.date}
                </small>
                <p class="card-text">${snippet}</p>
                <a href="${post.link}" class="card-link">
                  Read More <i class="fas fa-arrow-right"></i>
                </a>
              </div>
            `;
            postsContainer.appendChild(article);
          });
          console.log(`Loaded ${blogPosts.length} posts into #posts container from JSON.`);
        } else {
            postsContainer.innerHTML = '<p>No blog posts available yet.</p>';
            console.log("No blog posts found in JSON data.");
        }
      } catch (error) {
          console.error("Error fetching or processing blog posts:", error);
          postsContainer.innerHTML = '<p class="error-message">Could not load blog posts. Please try again later.</p>'; // Error message
      }
  
    } else {
      console.warn("Posts container (#posts) not found on this page.");
    }
  
  }); // End DOMContentLoaded