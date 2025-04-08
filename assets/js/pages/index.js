/**
 * ==========================================================================
 * Index Page Specific JavaScript (pages/index.js)
 *
 * Handles project filtering, GitHub integration display,
 * and loading latest blog posts.
 * ==========================================================================
 */

document.addEventListener("DOMContentLoaded", function () {
    console.log("Index Page JavaScript Initialized");
  
    // --- Blog Post Data ---
    // Consider moving this to a separate JSON file later for easier management
    // Duplicated from blog.js for now.
    const blogPosts = [
      {
        title: "The Journey Begins",
        content: "Welcome to my personal blog! This is where I'll share my thoughts, experiences, and insights.",
        date: "February 2, 2025",
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
      // Add more posts here if needed for testing 'latest' logic
    ];
  
    // --- Function to Load Latest Blog Posts ---
    function loadLatestPosts(numberOfPostsToShow = 3) { // Default to showing 3
      const postsContainer = document.getElementById("posts"); // Target container on index page
      if (!postsContainer) {
          console.warn("Latest posts container (#posts) not found on index page.");
          return;
      }
  
      // Clear any static placeholders in the #posts container on index.html
      postsContainer.innerHTML = '';
  
      // Get the latest posts (assuming newest are first in the array)
      // Use slice(0, N) to get the first N items. Reverse first if newest are last.
      // const latestPosts = blogPosts.reverse().slice(0, numberOfPostsToShow); // If newest are last
      const latestPosts = blogPosts.slice(0, numberOfPostsToShow); // If newest are first
  
      if (latestPosts.length > 0) {
        latestPosts.forEach((post) => {
          const article = document.createElement("article");
          article.classList.add("card", "post-preview"); // Use card styles
  
          // Format content snippet
          let snippet = post.content;
          const maxLength = 120; // Shorter snippet for index page?
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
         console.log(`Loaded ${latestPosts.length} latest posts into #posts container.`);
      } else {
         postsContainer.innerHTML = '<p>No recent blog posts available.</p>';
          console.log("No blog posts found to load.");
      }
    } // End loadLatestPosts
  
  
    // --- Project Filtering Logic ---
    function initProjectsFilter() {
      const filterButtons = document.querySelectorAll(".projects-filter .filter-btn");
      const projectCards = document.querySelectorAll(".projects-grid .project-card");
  
      if (!filterButtons.length || !projectCards.length) {
        if (!filterButtons.length) console.warn("Project filter buttons not found.");
        if (!projectCards.length) console.warn("Project cards not found.");
        return;
      }
  
      filterButtons.forEach(button => {
        button.addEventListener("click", function () {
          const filterValue = this.getAttribute("data-filter");
          filterButtons.forEach(btn => btn.classList.remove("active"));
          this.classList.add("active");
  
          projectCards.forEach(card => {
            const cardCategory = card.getAttribute("data-category");
            const shouldShow = (filterValue === "all" || cardCategory === filterValue);
            card.style.transition = 'opacity 0.3s ease-out, transform 0.3s ease-out';
            card.style.opacity = '0';
            card.style.transform = 'scale(0.95)';
            setTimeout(() => {
              if (shouldShow) {
                card.style.display = 'block';
                void card.offsetWidth;
                card.style.opacity = '1';
                card.style.transform = 'scale(1)';
              } else {
                card.style.display = 'none';
              }
            }, 300);
          });
        });
      });
  
       projectCards.forEach(card => {
          card.style.display = 'block';
          card.style.opacity = '1';
          card.style.transform = 'scale(1)';
       });
       console.log("Project filtering initialized.");
    } // End initProjectsFilter
  
    // --- GitHub Integration Display Logic ---
    function createContributionHeatmap() {
      const contributionSection = document.getElementById("github-contributions");
      if (!contributionSection) {
          // This section might not be present in the modified HTML, so just return silently or warn
          // console.warn("GitHub contribution section (#github-contributions) not found.");
          return;
      }
      const container = contributionSection.querySelector(".contribution-heatmap");
       if (!container) {
          console.warn("Contribution heatmap container (.contribution-heatmap) not found.");
          return;
      }
      container.innerHTML = `...`; // Same innerHTML structure as before
      populateContributionGrid();
       console.log("GitHub contribution heatmap initialized.");
    }
    function populateContributionGrid() { /* ... same as before ... */ }
    function generateActivityPattern(weeks, days) { /* ... same as before ... */ }
    function formatDate(date) { /* ... same as before ... */ }
    function showEnhancedTooltip(event) { /* ... same as before ... */ }
    function hideEnhancedTooltip(event) { /* ... same as before ... */ }
  
    // --- Initialize Index Page Features ---
    loadLatestPosts(3); // Load the 3 latest posts
    initProjectsFilter();
    // createContributionHeatmap(); // Call if the HTML section exists
  
  }); // End DOMContentLoaded
  
  // --- Include full implementations for GitHub functions if needed ---
  function populateContributionGrid() {
      const grid = document.getElementById("contribution-grid");
      if (!grid) return;
      grid.innerHTML = "";
      const numWeeks = 52; const numDays = 7; const today = new Date();
      const activityPattern = generateActivityPattern(numWeeks, numDays);
      for (let day = 0; day < numDays; day++) {
        for (let week = 0; week < numWeeks; week++) {
          const daysAgo = (numWeeks - week - 1) * 7 + (numDays - day - 1);
          const cellDate = new Date(today); cellDate.setDate(today.getDate() - daysAgo);
          const heatLevel = activityPattern[day][week];
          const cell = document.createElement("div"); cell.classList.add("grid-cell");
          if (heatLevel > 0) cell.classList.add(`heat-${heatLevel}`);
          cell.setAttribute("data-date", formatDate(cellDate));
          cell.setAttribute("data-count", heatLevel === 0 ? "0" : (heatLevel * 3 + Math.floor(Math.random() * 5)));
          // Add tooltip listeners if needed
          grid.appendChild(cell);
        }
      }
  }
  function generateActivityPattern(weeks, days) {
      const pattern = Array(days).fill().map(() => Array(weeks).fill(0));
      for (let day = 0; day < days; day++) {
        for (let week = 0; week < weeks; week++) {
          const recency = week / weeks; let probability = 0.15 + (recency * 0.65);
          if (day === 0 || day === 6) probability *= 0.6;
          const random = Math.random();
          if (random < probability * 0.2) pattern[day][week] = 4;
          else if (random < probability * 0.4) pattern[day][week] = 3;
          else if (random < probability * 0.6) pattern[day][week] = 2;
          else if (random < probability) pattern[day][week] = 1;
          else pattern[day][week] = 0;
        }
      } return pattern;
  }
  function formatDate(date) { const options = { weekday: 'long', year: 'numeric', month: 'long', day: 'numeric' }; return date.toLocaleDateString('en-US', options); }
  // --- End GitHub function implementations ---