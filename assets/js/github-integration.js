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
  

  // Enhanced GitHub Integration Script
document.addEventListener("DOMContentLoaded", function() {
    console.log("Enhanced GitHub Integration Script Loaded");
    
    // Initialize GitHub Contributions Heatmap
    initEnhancedContributionHeatmap();
    
    // Initialize Projects Filtering
    initProjectsFilter();
  });
  
  // Function to initialize the enhanced contribution heatmap
  function initEnhancedContributionHeatmap() {
    const contributionHeatmap = document.querySelector(".contribution-heatmap");
    
    if (!contributionHeatmap) return;
    
    // Clear any existing content in the heatmap
    const existingGrid = document.getElementById("contribution-grid");
    if (existingGrid) {
      existingGrid.innerHTML = '';
    }
    
    // Add labels for days of the week
    const daysOfWeek = ['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat'];
    const heatmapDays = document.createElement("div");
    heatmapDays.className = "heatmap-days";
    
    daysOfWeek.forEach(day => {
      const dayElement = document.createElement("div");
      dayElement.className = "heatmap-day";
      dayElement.textContent = day;
      heatmapDays.appendChild(dayElement);
    });
    
    // Create container for the heatmap
    const heatmapContainer = document.createElement("div");
    heatmapContainer.className = "heatmap-container";
    
    // Add week labels
    const weekLabels = document.createElement("div");
    weekLabels.className = "week-labels";
    
    ['', 'Mon', '', 'Wed', '', 'Fri', ''].forEach(label => {
      const labelElement = document.createElement("div");
      labelElement.className = "week-label";
      labelElement.textContent = label;
      weekLabels.appendChild(labelElement);
    });
    
    heatmapContainer.appendChild(weekLabels);
    
    // Create grid for contributions
    const grid = document.createElement("div");
    grid.id = "contribution-grid";
    grid.className = "heatmap-grid";
    heatmapContainer.appendChild(grid);
    
    // Add months labels
    const heatmapMonths = document.createElement("div");
    heatmapMonths.className = "heatmap-months";
    
    const months = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec'];
    months.forEach(month => {
      const monthElement = document.createElement("span");
      monthElement.textContent = month;
      heatmapMonths.appendChild(monthElement);
    });
    
    // Replace the existing content
    contributionHeatmap.innerHTML = '';
    
    // Add the header back
    const heatmapHeader = document.createElement("div");
    heatmapHeader.className = "heatmap-header";
    heatmapHeader.innerHTML = `
      <span>Contributions in the last year: <strong>854</strong></span>
      <div class="heatmap-legend">
        <span class="legend-label">Less</span>
        <ul class="legend-squares">
          <li class="heat-0"></li>
          <li class="heat-1"></li>
          <li class="heat-2"></li>
          <li class="heat-3"></li>
          <li class="heat-4"></li>
        </ul>
        <span class="legend-label">More</span>
      </div>
    `;
    
    contributionHeatmap.appendChild(heatmapHeader);
    contributionHeatmap.appendChild(heatmapDays);
    contributionHeatmap.appendChild(heatmapContainer);
    contributionHeatmap.appendChild(heatmapMonths);
    
    // Populate the grid with cells
    generateContributionData();
  }
  
  // Function to generate contribution data and populate the grid
  function generateContributionData() {
    const grid = document.getElementById("contribution-grid");
    if (!grid) return;
    
    // Generate data for 52 weeks (1 year)
    const numWeeks = 52;
    const numDays = 7;
    const totalDays = numWeeks * numDays;
    
    // Get current date
    const today = new Date();
    
    // Generate activity pattern (more activity in recent months, less in older months)
    for (let week = 0; week < numWeeks; week++) {
      for (let day = 0; day < numDays; day++) {
        // Calculate date for this cell
        const cellDate = new Date(today);
        cellDate.setDate(today.getDate() - (totalDays - (week * numDays + day)));
        
        // Generate activity level (0-4)
        // More recent dates have higher probability of more activity
        let heatLevel;
        const recency = week / numWeeks; // 0 = oldest, 1 = newest
        const randomFactor = Math.random();
        
        if (recency > 0.75) {
          // Last quarter: higher activity
          heatLevel = Math.floor(randomFactor * 5);
          if (randomFactor > 0.3) heatLevel = Math.max(1, heatLevel); // 70% chance of at least level 1
        } else if (recency > 0.5) {
          // 3rd quarter: medium activity
          heatLevel = Math.floor(randomFactor * 4);
        } else if (recency > 0.25) {
          // 2nd quarter: lower activity
          heatLevel = Math.floor(randomFactor * 3);
        } else {
          // 1st quarter: sparse activity
          heatLevel = Math.floor(randomFactor * 2);
        }
        
        // Weekend days (0=Sunday, 6=Saturday) have less activity
        if ((day === 0 || day === 6) && Math.random() > 0.3) {
          heatLevel = Math.max(0, heatLevel - 1);
        }
        
        // Create the cell
        const cell = document.createElement("div");
        cell.classList.add("grid-cell");
        
        if (heatLevel > 0) {
          cell.classList.add(`heat-${heatLevel}`);
        }
        
        // Add data attributes for tooltip
        cell.setAttribute("data-date", formatDate(cellDate));
        cell.setAttribute("data-count", heatLevel === 0 ? "0" : Math.floor(Math.random() * 10 + heatLevel * 2));
        
        // Add tooltip functionality
        cell.addEventListener("mouseover", showEnhancedTooltip);
        cell.addEventListener("mouseout", hideEnhancedTooltip);
        
        grid.appendChild(cell);
      }
    }
  }
  
  // Helper function to format date
  function formatDate(date) {
    const options = { weekday: 'long', year: 'numeric', month: 'long', day: 'numeric' };
    return date.toLocaleDateString('en-US', options);
  }
  
  // Functions for enhanced tooltip
  function showEnhancedTooltip(event) {
    const cell = event.target;
    const date = cell.getAttribute("data-date");
    const count = cell.getAttribute("data-count");
    
    // Remove any existing tooltips
    const existingTooltips = document.querySelectorAll(".heatmap-tooltip");
    existingTooltips.forEach(tooltip => tooltip.remove());
    
    // Create tooltip
    const tooltip = document.createElement("div");
    tooltip.classList.add("heatmap-tooltip");
    tooltip.textContent = `${count} contributions on ${date}`;
    
    // Position the tooltip
    const rect = cell.getBoundingClientRect();
    const scrollTop = window.pageYOffset || document.documentElement.scrollTop;
    
    tooltip.style.left = `${rect.left + rect.width / 2}px`;
    tooltip.style.top = `${rect.top + scrollTop - 40}px`;
    tooltip.style.transform = "translateX(-50%)";
    
    document.body.appendChild(tooltip);
    cell.setAttribute("data-tooltip-id", Date.now());
    tooltip.id = cell.getAttribute("data-tooltip-id");
  }
  
  function hideEnhancedTooltip(event) {
    const cell = event.target;
    const tooltipId = cell.getAttribute("data-tooltip-id");
    if (tooltipId) {
      const tooltip = document.getElementById(tooltipId);
      if (tooltip) {
        tooltip.remove();
      }
    }
  }
  
  // Function to initialize projects filtering with animation
  function initProjectsFilter() {
    const filterButtons = document.querySelectorAll(".filter-btn");
    const projectCards = document.querySelectorAll(".project-card");
    
    if (!filterButtons.length || !projectCards.length) return;
    
    // Enhance filter buttons
    filterButtons.forEach(button => {
      // Add hover effect
      button.addEventListener("mouseenter", function() {
        if (!this.classList.contains("active")) {
          this.style.transform = "translateY(-3px)";
          this.style.boxShadow = "0 5px 15px rgba(0, 0, 0, 0.1)";
        }
      });
      
      button.addEventListener("mouseleave", function() {
        if (!this.classList.contains("active")) {
          this.style.transform = "";
          this.style.boxShadow = "";
        }
      });
      
      // Add click behavior with animation
      button.addEventListener("click", function() {
        const filterValue = this.getAttribute("data-filter");
        
        // Remove active class from all buttons
        filterButtons.forEach(btn => {
          btn.classList.remove("active");
          btn.style.transform = "";
          btn.style.boxShadow = "";
        });
        
        // Add active class to clicked button
        this.classList.add("active");
        this.style.transform = "translateY(-3px)";
        this.style.boxShadow = "0 5px 15px rgba(0, 0, 0, 0.1)";
        
        // Animate cards
        projectCards.forEach(card => {
          // First set all to fade out
          card.style.opacity = "0.5";
          card.style.transform = "scale(0.95)";
          card.style.transition = "all 0.3s ease";
          
          // Then filter and animate the visible ones
          setTimeout(() => {
            if (filterValue === "all" || card.getAttribute("data-category") === filterValue) {
              card.style.display = "block";
              setTimeout(() => {
                card.style.opacity = "1";
                card.style.transform = "scale(1)";
              }, 10);
            } else {
              card.style.display = "none";
            }
          }, 300);
        });
      });
    });
    
    // Initialize all cards as visible
    projectCards.forEach(card => {
      card.style.opacity = "1";
      card.style.transform = "scale(1)";
      card.style.transition = "all 0.3s ease";
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