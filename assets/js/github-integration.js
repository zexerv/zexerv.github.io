// Fixed GitHub Integration Script
document.addEventListener("DOMContentLoaded", function() {
    console.log("Fixed GitHub Integration Script Loaded");
    
    // Initialize GitHub Contributions Heatmap
    createContributionHeatmap();
    
    // Initialize Projects Filtering
    initProjectsFilter();
  });
  
  // Function to create the contribution heatmap from scratch
  function createContributionHeatmap() {
    const contributionSection = document.getElementById("github-contributions");
    if (!contributionSection) return;
    
    // Find the container for the heatmap
    const container = contributionSection.querySelector(".contribution-heatmap");
    if (!container) return;
    
    // Clear existing content and rebuild it
    container.innerHTML = `
      <div class="heatmap-header">
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
      </div>
      <div class="heatmap-days">
        <div class="heatmap-day">Sun</div>
        <div class="heatmap-day">Mon</div>
        <div class="heatmap-day">Tue</div>
        <div class="heatmap-day">Wed</div>
        <div class="heatmap-day">Thu</div>
        <div class="heatmap-day">Fri</div>
        <div class="heatmap-day">Sat</div>
      </div>
      <div class="heatmap-container">
        <div class="week-labels">
          <div class="week-label"></div>
          <div class="week-label">Mon</div>
          <div class="week-label"></div>
          <div class="week-label">Wed</div>
          <div class="week-label"></div>
          <div class="week-label">Fri</div>
          <div class="week-label"></div>
        </div>
        <div id="contribution-grid" class="heatmap-grid"></div>
      </div>
      <div class="heatmap-months">
        <span>Jan</span>
        <span>Feb</span>
        <span>Mar</span>
        <span>Apr</span>
        <span>May</span>
        <span>Jun</span>
        <span>Jul</span>
        <span>Aug</span>
        <span>Sep</span>
        <span>Oct</span>
        <span>Nov</span>
        <span>Dec</span>
      </div>
    `;
    
    // Now populate the grid with cells
    populateContributionGrid();
  }
  
  // Function to populate the contribution grid with data
  function populateContributionGrid() {
    const grid = document.getElementById("contribution-grid");
    if (!grid) return;
    
    // Clear any existing content
    grid.innerHTML = "";
    
    // Generate data for 52 weeks (1 year)
    const numWeeks = 52;
    const numDays = 7;
    
    // Get current date
    const today = new Date();
    
    // Create a pattern of activity (you can customize this)
    const activityPattern = generateActivityPattern(numWeeks, numDays);
    
    // Create cells for each day
    for (let day = 0; day < numDays; day++) {
      for (let week = 0; week < numWeeks; week++) {
        // Calculate date for this cell
        const daysAgo = (numWeeks - week - 1) * 7 + (numDays - day - 1);
        const cellDate = new Date(today);
        cellDate.setDate(today.getDate() - daysAgo);
        
        // Get activity level from pattern
        const heatLevel = activityPattern[day][week];
        
        // Create the cell
        const cell = document.createElement("div");
        cell.classList.add("grid-cell");
        
        if (heatLevel > 0) {
          cell.classList.add(`heat-${heatLevel}`);
        }
        
        // Add data attributes for tooltip
        cell.setAttribute("data-date", formatDate(cellDate));
        cell.setAttribute("data-count", heatLevel === 0 ? "0" : (heatLevel * 3 + Math.floor(Math.random() * 5)));
        
        // Add tooltip functionality
        cell.addEventListener("mouseover", showEnhancedTooltip);
        cell.addEventListener("mouseout", hideEnhancedTooltip);
        
        grid.appendChild(cell);
      }
    }
  }
  
  // Function to generate a realistic activity pattern
  function generateActivityPattern(weeks, days) {
    const pattern = Array(days).fill().map(() => Array(weeks).fill(0));
    
    // Create a realistic pattern with more activity in recent months
    for (let day = 0; day < days; day++) {
      for (let week = 0; week < weeks; week++) {
        // Calculate probability based on recency (higher for recent weeks)
        const recency = week / weeks; // 0 = oldest, 1 = newest
        let probability;
        
        if (recency > 0.8) {         // Last 20% of time (very recent)
          probability = 0.8;
        } else if (recency > 0.6) {  // 60-80% of time
          probability = 0.7;
        } else if (recency > 0.4) {  // 40-60% of time
          probability = 0.5;
        } else if (recency > 0.2) {  // 20-40% of time
          probability = 0.3;
        } else {                     // First 20% of time (oldest)
          probability = 0.15;
        }
        
        // Weekend adjustment (less activity on weekends)
        if (day === 0 || day === 6) {
          probability *= 0.6;
        }
        
        // Randomly determine activity level based on probability
        const random = Math.random();
        
        if (random < probability * 0.2) {
          pattern[day][week] = 4; // Very high activity (darkest green)
        } else if (random < probability * 0.4) {
          pattern[day][week] = 3; // High activity
        } else if (random < probability * 0.6) {
          pattern[day][week] = 2; // Medium activity
        } else if (random < probability) {
          pattern[day][week] = 1; // Low activity
        } else {
          pattern[day][week] = 0; // No activity
        }
      }
    }
    
    return pattern;
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