/**
 * ==========================================================================
 * Index Page Specific JavaScript (pages/index.js) - UPDATED TO FETCH JSON
 *
 * Handles project filtering, GitHub integration display,
 * and loading latest blog posts from JSON.
 * ==========================================================================
 */

document.addEventListener("DOMContentLoaded", async function () { // Add async
    console.log("Index Page JavaScript Initialized");
  
    const blogDataUrl = '../../assets/data/blogPosts.json'; // Path relative to index.js
    let blogPosts = []; // Variable to hold fetched data
  
    // --- Fetch Blog Post Data ---
    async function fetchBlogData() {
        try {
            const response = await fetch(blogDataUrl);
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            blogPosts = await response.json();
            console.log("Blog post data fetched successfully.");
            // Call function to load posts after data is fetched
            loadLatestPosts(3); // Load the 3 latest posts
        } catch (error) {
            console.error("Failed to fetch blog data:", error);
            // Optionally display an error in the #posts container
            const postsContainer = document.getElementById("posts");
            if (postsContainer) {
                postsContainer.innerHTML = '<p><small>Could not load recent posts.</small></p>';
            }
        }
    }
  
    // --- Function to Load Latest Blog Posts ---
    // Now uses the globally fetched blogPosts array
    function loadLatestPosts(numberOfPostsToShow = 3) {
      const postsContainer = document.getElementById("posts"); // Target container on index page
      if (!postsContainer) {
          console.warn("Latest posts container (#posts) not found on index page.");
          return;
      }
      postsContainer.innerHTML = ''; // Clear any previous content/placeholders
  
      // Assuming newest posts are first in the JSON/array
      const latestPosts = blogPosts.slice(0, numberOfPostsToShow);
  
      if (latestPosts.length > 0) {
        latestPosts.forEach((post) => {
          const article = document.createElement("article");
          article.classList.add("card", "post-preview");
  
          let snippet = post.content;
          const maxLength = 120;
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
      } else if (blogPosts.length > 0) {
          // Handle case where data loaded but slice result is empty (e.g., asking for 0 posts)
          postsContainer.innerHTML = '<p>No recent posts to display.</p>';
      } else {
          // Data might not have loaded yet or is empty, message handled in fetchBlogData catch block
      }
    } // End loadLatestPosts
  
  
    // --- Project Filtering Logic ---
    function initProjectsFilter() { /* ... same as before ... */ }
  
    // --- GitHub Integration Display Logic ---
    function createContributionHeatmap() { /* ... same as before ... */ }
    function populateContributionGrid() { /* ... same as before ... */ }
    function generateActivityPattern(weeks, days) { /* ... same as before ... */ }
    function formatDate(date) { /* ... same as before ... */ }
    function showEnhancedTooltip(event) { /* ... same as before ... */ }
    function hideEnhancedTooltip(event) { /* ... same as before ... */ }
  
    // --- Initialize Index Page Features ---
    await fetchBlogData(); // Fetch data first
    initProjectsFilter();
    // createContributionHeatmap(); // Call if the HTML section exists
  
  }); // End DOMContentLoaded
  
  // --- Include full implementations for GitHub/Project functions if needed ---
  function initProjectsFilter() {
      const filterButtons = document.querySelectorAll(".projects-filter .filter-btn");
      const projectCards = document.querySelectorAll(".projects-grid .project-card");
      if (!filterButtons.length || !projectCards.length) { if (!filterButtons.length) console.warn("Project filter buttons not found."); if (!projectCards.length) console.warn("Project cards not found."); return; }
      filterButtons.forEach(button => {
        button.addEventListener("click", function () {
          const filterValue = this.getAttribute("data-filter");
          filterButtons.forEach(btn => btn.classList.remove("active")); this.classList.add("active");
          projectCards.forEach(card => {
            const cardCategory = card.getAttribute("data-category"); const shouldShow = (filterValue === "all" || cardCategory === filterValue);
            card.style.transition = 'opacity 0.3s ease-out, transform 0.3s ease-out'; card.style.opacity = '0'; card.style.transform = 'scale(0.95)';
            setTimeout(() => {
              if (shouldShow) { card.style.display = 'block'; void card.offsetWidth; card.style.opacity = '1'; card.style.transform = 'scale(1)'; }
              else { card.style.display = 'none'; }
            }, 300); }); }); });
       projectCards.forEach(card => { card.style.display = 'block'; card.style.opacity = '1'; card.style.transform = 'scale(1)'; });
       console.log("Project filtering initialized.");
  }
  function createContributionHeatmap() {
      const contributionSection = document.getElementById("github-contributions"); if (!contributionSection) return;
      const container = contributionSection.querySelector(".contribution-heatmap"); if (!container) { console.warn("Contribution heatmap container not found."); return; }
      container.innerHTML = `<div class="heatmap-header"><span>Contributions last year: <strong>854</strong></span><div class="heatmap-legend"><span class="legend-label">Less</span><ul class="legend-squares"><li class="heat-0"></li><li class="heat-1"></li><li class="heat-2"></li><li class="heat-3"></li><li class="heat-4"></li></ul><span class="legend-label">More</span></div></div><div class="heatmap-container"><div class="week-labels"><div class="week-label"></div><div class="week-label">Mon</div><div class="week-label"></div><div class="week-label">Wed</div><div class="week-label"></div><div class="week-label">Fri</div><div class="week-label"></div></div><div id="contribution-grid" class="heatmap-grid"></div></div><div class="heatmap-months"><span>Jan</span><span>Feb</span><span>Mar</span><span>Apr</span><span>May</span><span>Jun</span><span>Jul</span><span>Aug</span><span>Sep</span><span>Oct</span><span>Nov</span><span>Dec</span></div>`;
      populateContributionGrid(); console.log("GitHub contribution heatmap initialized.");
  }
  function populateContributionGrid() {
      const grid = document.getElementById("contribution-grid"); if (!grid) return; grid.innerHTML = "";
      const numWeeks = 52; const numDays = 7; const today = new Date(); const activityPattern = generateActivityPattern(numWeeks, numDays);
      for (let day = 0; day < numDays; day++) { for (let week = 0; week < numWeeks; week++) {
          const daysAgo = (numWeeks - week - 1) * 7 + (numDays - day - 1); const cellDate = new Date(today); cellDate.setDate(today.getDate() - daysAgo); const heatLevel = activityPattern[day][week];
          const cell = document.createElement("div"); cell.classList.add("grid-cell"); if (heatLevel > 0) cell.classList.add(`heat-${heatLevel}`);
          cell.setAttribute("data-date", formatDate(cellDate)); cell.setAttribute("data-count", heatLevel === 0 ? "0" : (heatLevel * 3 + Math.floor(Math.random() * 5)));
          grid.appendChild(cell); } }
  }
  function generateActivityPattern(weeks, days) { const pattern = Array(days).fill().map(() => Array(weeks).fill(0)); for (let day = 0; day < days; day++) { for (let week = 0; week < weeks; week++) { const recency = week / weeks; let probability = 0.15 + (recency * 0.65); if (day === 0 || day === 6) probability *= 0.6; const random = Math.random(); if (random < probability * 0.2) pattern[day][week] = 4; else if (random < probability * 0.4) pattern[day][week] = 3; else if (random < probability * 0.6) pattern[day][week] = 2; else if (random < probability) pattern[day][week] = 1; else pattern[day][week] = 0; } } return pattern; }
  function formatDate(date) { const options = { weekday: 'long', year: 'numeric', month: 'long', day: 'numeric' }; return date.toLocaleDateString('en-US', options); }
  function showEnhancedTooltip(event) { /* TODO */ } function hideEnhancedTooltip(event) { /* TODO */ }
  // --- End full implementations ---