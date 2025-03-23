// GitHub Integration Script
document.addEventListener("DOMContentLoaded", function() {
    console.log("GitHub Integration Script Loaded");
    
    // Initialize GitHub Contributions Heatmap
    initContributionHeatmap();
    
    // Initialize Projects Filtering
    initProjectsFilter();
  });
  
  // Function to initialize the contribution heatmap
  function initContributionHeatmap() {
    const contributionGrid = document.getElementById("contribution-grid");
    
    if (!contributionGrid) return;
    
    // This would normally be fetched from GitHub API or pre-generated data
    // For demo purposes, we'll generate random contribution data
    const days = 7 * 52; // 1 year of data (7 days * 52 weeks)
    
    // Create cells for each day
    for (let i = 0; i < days; i++) {
      const cell = document.createElement("div");
      cell.classList.add("grid-cell");
      
      // Randomly assign heat level (0-4)
      const heatLevel = Math.floor(Math.random() * 5);
      if (heatLevel > 0) {
        cell.classList.add(`heat-${heatLevel}`);
      }
      
      // Add data attributes for tooltip (would be actual dates and contribution counts)
      const today = new Date();
      const date = new Date(today);
      date.setDate(today.getDate() - (days - i));
      
      cell.setAttribute("data-date", formatDate(date));
      cell.setAttribute("data-count", Math.floor(Math.random() * 10));
      
      // Add tooltip functionality
      cell.addEventListener("mouseover", showTooltip);
      cell.addEventListener("mouseout", hideTooltip);
      
      contributionGrid.appendChild(cell);
    }
  }
  
  // Helper function to format date
  function formatDate(date) {
    const options = { year: 'numeric', month: 'short', day: 'numeric' };
    return date.toLocaleDateString('en-US', options);
  }
  
  // Functions for tooltip
  function showTooltip(event) {
    const cell = event.target;
    const date = cell.getAttribute("data-date");
    const count = cell.getAttribute("data-count");
    
    // Create tooltip
    const tooltip = document.createElement("div");
    tooltip.classList.add("heatmap-tooltip");
    tooltip.textContent = `${count} contributions on ${date}`;
    
    // Position the tooltip
    const rect = cell.getBoundingClientRect();
    tooltip.style.position = "absolute";
    tooltip.style.left = `${rect.left}px`;
    tooltip.style.top = `${rect.top - 30}px`;
    tooltip.style.backgroundColor = "#333";
    tooltip.style.color = "white";
    tooltip.style.padding = "5px 10px";
    tooltip.style.borderRadius = "4px";
    tooltip.style.fontSize = "12px";
    tooltip.style.zIndex = "100";
    
    document.body.appendChild(tooltip);
    cell.setAttribute("data-tooltip-id", Date.now());
    tooltip.id = cell.getAttribute("data-tooltip-id");
  }
  
  function hideTooltip(event) {
    const cell = event.target;
    const tooltipId = cell.getAttribute("data-tooltip-id");
    if (tooltipId) {
      const tooltip = document.getElementById(tooltipId);
      if (tooltip) {
        tooltip.remove();
      }
    }
  }
  
  // Function to initialize projects filtering
  function initProjectsFilter() {
    const filterButtons = document.querySelectorAll(".filter-btn");
    const projectCards = document.querySelectorAll(".project-card");
    
    if (!filterButtons.length || !projectCards.length) return;
    
    filterButtons.forEach(button => {
      button.addEventListener("click", function() {
        // Remove active class from all buttons
        filterButtons.forEach(btn => btn.classList.remove("active"));
        
        // Add active class to clicked button
        this.classList.add("active");
        
        // Get filter value
        const filterValue = this.getAttribute("data-filter");
        
        // Filter projects
        projectCards.forEach(card => {
          if (filterValue === "all" || card.getAttribute("data-category") === filterValue) {
            card.style.display = "block";
          } else {
            card.style.display = "none";
          }
        });
      });
    });
  }
  
  // You could add functions here to fetch actual GitHub data
  // using the GitHub API if you implement a backend service
  // Example:
  /*
  async function fetchGitHubData(username) {
    try {
      // This would require a backend proxy to avoid CORS issues and to hide your GitHub token
      const response = await fetch(`/api/github/${username}`);
      const data = await response.json();
      return data;
    } catch (error) {
      console.error("Error fetching GitHub data:", error);
      return null;
    }
  }
  */