// --- DOM Elements & Global State ---
const canvas = document.getElementById('segmentation-canvas');
const ctx = canvas.getContext('2d');
const runBtn = document.getElementById('run-segmentation-btn');
const clearBtn = document.getElementById('clear-canvas-btn');
const infoDisplay = document.getElementById('info-display');
const downsampleSlider = document.getElementById('downsample-slider');
const downsampleValueSpan = document.getElementById('downsample-value');

let isDrawing = false;
let trajectories = []; // Array of arrays of points, e.g., [[{x, y}, ...], [{x, y}, ...]]
let currentTrajectory = [];

// --- Canvas & Drawing Setup ---

function resizeCanvas() {
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    ctx.scale(dpr, dpr);
    redrawAll(); // Redraw content after resizing
}

function getMousePos(evt) {
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / (rect.width * (window.devicePixelRatio || 1));
    const scaleY = canvas.height / (rect.height * (window.devicePixelRatio || 1));
    return {
        x: (evt.clientX - rect.left) * scaleX,
        y: (evt.clientY - rect.top) * scaleY,
    };
}

function startDrawing(e) {
    isDrawing = true;
    currentTrajectory = [getMousePos(e)];
    trajectories.push(currentTrajectory);
    ctx.beginPath();
    ctx.moveTo(currentTrajectory[0].x, currentTrajectory[0].y);
    ctx.strokeStyle = '#E2E8F0'; // Light gray for drawing
    ctx.lineWidth = 2;
}

function draw(e) {
    if (!isDrawing) return;
    const pos = getMousePos(e);
    currentTrajectory.push(pos);
    ctx.lineTo(pos.x, pos.y);
    ctx.stroke();
}

function stopDrawing() {
    if (!isDrawing) return;
    ctx.closePath();
    isDrawing = false;
}

function redrawAll() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.strokeStyle = '#A0AEC0'; // Muted gray for existing lines
    ctx.lineWidth = 1.5;
    trajectories.forEach(traj => {
        if (traj.length < 2) return;
        ctx.beginPath();
        ctx.moveTo(traj[0].x, traj[0].y);
        for (let i = 1; i < traj.length; i++) {
            ctx.lineTo(traj[i].x, traj[i].y);
        }
        ctx.stroke();
        ctx.closePath();
    });
}

// --- Algorithm Implementations ---

// 1. Downsampling
function downsample(trajectory, factor) {
    if (factor <= 1 || trajectory.length < factor) {
        return trajectory;
    }
    const downsampled = [];
    for (let i = 0; i < trajectory.length; i += factor) {
        const end = Math.min(i + factor, trajectory.length);
        const chunk = trajectory.slice(i, end);
        const avg = chunk.reduce((acc, p) => ({ x: acc.x + p.x, y: acc.y + p.y }), { x: 0, y: 0 });
        avg.x /= chunk.length;
        avg.y /= chunk.length;
        downsampled.push(avg);
    }
    return downsampled;
}


// 2. Alignment (DTW)
function euclideanDistance(p1, p2) {
    return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
}

function dtw(refSeq, trajSeq) {
    const n = refSeq.length;
    const m = trajSeq.length;
    const costMatrix = Array(n).fill(null).map(() => Array(m).fill(Infinity));

    costMatrix[0][0] = euclideanDistance(refSeq[0], trajSeq[0]);

    for (let i = 1; i < n; i++) {
        costMatrix[i][0] = costMatrix[i-1][0] + euclideanDistance(refSeq[i], trajSeq[0]);
    }
    for (let j = 1; j < m; j++) {
        costMatrix[0][j] = costMatrix[0][j-1] + euclideanDistance(refSeq[0], trajSeq[j]);
    }

    for (let i = 1; i < n; i++) {
        for (let j = 1; j < m; j++) {
            const cost = euclideanDistance(refSeq[i], trajSeq[j]);
            costMatrix[i][j] = cost + Math.min(
                costMatrix[i-1][j],      // Deletion
                costMatrix[i][j-1],      // Insertion
                costMatrix[i-1][j-1]     // Match
            );
        }
    }

    // Backtrack path
    let path = [];
    let i = n - 1;
    let j = m - 1;
    path.push([i, j]);
    while (i > 0 || j > 0) {
        if (i > 0 && j > 0) {
            const minPrev = Math.min(costMatrix[i-1][j], costMatrix[i][j-1], costMatrix[i-1][j-1]);
            if (minPrev === costMatrix[i-1][j-1]) { i--; j--; }
            else if (minPrev === costMatrix[i-1][j]) { i--; }
            else { j--; }
        } else if (i > 0) {
            i--;
        } else {
            j--;
        }
        path.push([i, j]);
    }
    path.reverse();
    return path;
}


function alignTrajectories(trajectories) {
    if (trajectories.length <= 1) {
        return trajectories;
    }
    const lengths = trajectories.map(t => t.length);
    const refIndex = lengths.indexOf(Math.max(...lengths));
    const refTraj = trajectories[refIndex];
    const alignedTrajectories = [refTraj];

    for (let i = 0; i < trajectories.length; i++) {
        if (i === refIndex) continue;
        const currentTraj = trajectories[i];
        const path = dtw(refTraj, currentTraj);

        const warpedTraj = [];
        let lastTrajIndex = 0;
        const refToTrajMap = {};
        path.forEach(([refI, trajI]) => {
            if (!(refI in refToTrajMap)) refToTrajMap[refI] = trajI;
        });

        for (let j = 0; j < refTraj.length; j++) {
             if (j in refToTrajMap) {
                 lastTrajIndex = refToTrajMap[j];
             }
             warpedTraj.push(currentTraj[lastTrajIndex]);
        }
        alignedTrajectories.push(warpedTraj);
    }
    return alignedTrajectories;
}

// 3. Tube Calculation
function calculateTube(alignedTrajectories) {
    const n = alignedTrajectories[0].length;
    const tubeMin = Array(n).fill(null).map(() => ({x: Infinity, y: Infinity}));
    const tubeMax = Array(n).fill(null).map(() => ({x: -Infinity, y: -Infinity}));

    for(let i = 0; i < n; i++) {
        for(let j = 0; j < alignedTrajectories.length; j++) {
            const point = alignedTrajectories[j][i];
            if (!point) continue;
            tubeMin[i].x = Math.min(tubeMin[i].x, point.x);
            tubeMin[i].y = Math.min(tubeMin[i].y, point.y);
            tubeMax[i].x = Math.max(tubeMax[i].x, point.x);
            tubeMax[i].y = Math.max(tubeMax[i].y, point.y);
        }
    }
    return { tubeMin, tubeMax };
}


// 4. Segmentation Algorithms

// SEGDP (Slow DP)
function getSegmentCost(start, end, tubeMin, tubeMax) {
    const segmentLen = end - start + 1;
    if (segmentLen <= 1) return 0;
    let cost = 0;

    for (const dim of ['x', 'y']) {
        const min_start_val = tubeMin[start][dim];
        const min_end_val = tubeMin[end][dim];
        const max_start_val = tubeMax[start][dim];
        const max_end_val = tubeMax[end][dim];

        const min_slope = (min_end_val - min_start_val) / (segmentLen - 1);
        const max_slope = (max_end_val - max_start_val) / (segmentLen - 1);

        for (let i = 0; i < segmentLen; i++) {
            const t = start + i;
            const approx_min = min_start_val + min_slope * i;
            const approx_max = max_start_val + max_slope * i;
            cost += Math.pow(tubeMin[t][dim] - approx_min, 2);
            cost += Math.pow(tubeMax[t][dim] - approx_max, 2);
        }
    }
    return cost;
}

function runSEGDP(tubeMin, tubeMax, lambda = 1000) {
    const N = tubeMin.length;
    const MAX_SEGMENTS = Math.min(N, 20); // Limit max segments for performance

    const costs = Array(N).fill(null).map(() => Array(N).fill(0));
    for (let i = 0; i < N; i++) {
        for (let j = i; j < N; j++) {
            costs[i][j] = getSegmentCost(i, j, tubeMin, tubeMax);
        }
    }

    const dp = Array(N).fill(null).map(() => Array(MAX_SEGMENTS + 1).fill(Infinity));
    const bp = Array(N).fill(null).map(() => Array(MAX_SEGMENTS + 1).fill(-1));

    for (let t = 0; t < N; t++) {
        dp[t][1] = costs[0][t];
        bp[t][1] = 0;
    }

    for (let m = 2; m <= MAX_SEGMENTS; m++) {
        for (let t = 0; t < N; t++) {
            for (let j = 0; j < t; j++) {
                const currentCost = dp[j][m-1] + costs[j+1][t];
                if (currentCost < dp[t][m]) {
                    dp[t][m] = currentCost;
                    bp[t][m] = j + 1;
                }
            }
        }
    }

    let bestNumSegments = -1;
    let minTotalCost = Infinity;

    for (let m = 1; m <= MAX_SEGMENTS; m++) {
        const totalCost = dp[N-1][m] + lambda * m;
        if (totalCost < minTotalCost) {
            minTotalCost = totalCost;
            bestNumSegments = m;
        }
    }

    const changepoints = [N];
    let current_t = N - 1;
    let current_m = bestNumSegments;
    while (current_m > 0) {
        let startOfSeg = bp[current_t][current_m];
        if (startOfSeg === -1) break;
        changepoints.push(startOfSeg);
        current_t = startOfSeg - 1;
        current_m--;
    }
    return changepoints.sort((a,b) => a-b).filter((v,i,a) => a.indexOf(v)===i);
}


// FastSEGDP (PELT)
function linearFit(points) {
    const n = points.length;
    if (n === 0) return { slope: 0, intercept: 0, error: 0 };
    let sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    for (let i = 0; i < n; i++) {
        sum_x += i;
        sum_y += points[i];
        sum_xy += i * points[i];
        sum_xx += i * i;
    }
    const slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    const intercept = (sum_y - slope * sum_x) / n;
    let error = 0;
    for (let i = 0; i < n; i++) {
        error += Math.pow(points[i] - (slope * i + intercept), 2);
    }
    return { slope, intercept, error };
}

function getSegmentCostOLS(start, end, tubeMin, tubeMax) {
    const segmentMinX = tubeMin.slice(start, end + 1).map(p => p.x);
    const segmentMinY = tubeMin.slice(start, end + 1).map(p => p.y);
    const segmentMaxX = tubeMax.slice(start, end + 1).map(p => p.x);
    const segmentMaxY = tubeMax.slice(start, end + 1).map(p => p.y);

    const cost = linearFit(segmentMinX).error +
                 linearFit(segmentMinY).error +
                 linearFit(segmentMaxX).error +
                 linearFit(segmentMaxY).error;
    return cost;
}

function runFastSEGDP(tubeMin, tubeMax, lambda = 1000000) { // PELT needs higher lambda
    const N = tubeMin.length;
    const F = Array(N + 1).fill(Infinity);
    const P = Array(N + 1).fill(0); // Pointers for backtracking
    F[0] = -lambda;
    let R = [0]; // Set of candidates

    const costCache = {};
    const getCost = (s, e) => {
        const key = `${s}-${e}`;
        if (key in costCache) return costCache[key];
        const cost = getSegmentCostOLS(s, e, tubeMin, tubeMax);
        costCache[key] = cost;
        return cost;
    };

    for (let t = 1; t <= N; t++) {
        let best_tau_for_t = 0;
        let min_cost_for_t = Infinity;

        for (const tau of R) {
            const cost = F[tau] + getCost(tau, t - 1) + lambda;
            if (cost < min_cost_for_t) {
                min_cost_for_t = cost;
                best_tau_for_t = tau;
            }
        }
        F[t] = min_cost_for_t;
        P[t] = best_tau_for_t;

        // Pruning step
        R = R.filter(tau => F[tau] + getCost(tau, t - 1) <= F[t]);
        R.push(t);
    }

    const changepoints = [N];
    let current_t = N;
    while(current_t > 0) {
        let cp = P[current_t];
        changepoints.push(cp);
        current_t = cp;
    }
    return changepoints.sort((a,b) => a-b).filter((v,i,a) => a.indexOf(v)===i);
}


// --- Visualization ---

function drawTube(tubeMin, tubeMax) {
    ctx.fillStyle = 'rgba(100, 116, 139, 0.3)'; // Slate color, semi-transparent
    ctx.beginPath();
    ctx.moveTo(tubeMax[0].x, tubeMax[0].y);
    for (let i = 1; i < tubeMax.length; i++) {
        ctx.lineTo(tubeMax[i].x, tubeMax[i].y);
    }
    for (let i = tubeMin.length - 1; i >= 0; i--) {
        ctx.lineTo(tubeMin[i].x, tubeMin[i].y);
    }
    ctx.closePath();
    ctx.fill();
}
// REPLACE the existing drawSegmentation function with this one.

function drawSegmentation(changepoints, tubeMin, tubeMax, color1, color2, isFast) {
    ctx.lineWidth = 2.5;

    for(let i=0; i < changepoints.length - 1; i++) {
        let start = changepoints[i];
        let end = changepoints[i+1];
        if(isFast) end -= 1; // PELT gives start of next, DP gives end of current
        if (end < 0) end = 0;
        if (end <= start) continue;

        if (isFast) {
            // --- OLS fit for FastSEGDP visualization ---
            const pointsMin = tubeMin.slice(start, end + 1);
            const pointsMax = tubeMax.slice(start, end + 1);

            // Fit Y over X for visualization
            const fitYoverX_min = linearFitXY(pointsMin);
            const fitYoverX_max = linearFitXY(pointsMax);

            // Get the actual start and end x-coordinates for drawing the line
            const startX = tubeMin[start].x;
            const endX = tubeMin[end].x;

            // Draw the fitted line for the MIN boundary
            ctx.strokeStyle = color1;
            ctx.beginPath();
            ctx.moveTo(startX, fitYoverX_min.slope * startX + fitYoverX_min.intercept);
            ctx.lineTo(endX, fitYoverX_min.slope * endX + fitYoverX_min.intercept);
            ctx.stroke();

            // Draw the fitted line for the MAX boundary
            ctx.strokeStyle = color2;
            ctx.beginPath();
            ctx.moveTo(startX, fitYoverX_max.slope * startX + fitYoverX_max.intercept);
            ctx.lineTo(endX, fitYoverX_max.slope * endX + fitYoverX_max.intercept);
            ctx.stroke();

        } else {
             // --- Trapezoid for SEGDP visualization ---
            ctx.strokeStyle = color1;
            ctx.beginPath();
            ctx.moveTo(tubeMin[start].x, tubeMin[start].y);
            ctx.lineTo(tubeMin[end].x, tubeMin[end].y);
            ctx.stroke();

            ctx.strokeStyle = color2;
            ctx.beginPath();
            ctx.moveTo(tubeMax[start].x, tubeMax[start].y);
            ctx.lineTo(tubeMax[end].x, tubeMax[end].y);
            ctx.stroke();
        }
    }
}

// ADD this new helper function right below the linearFit function.
function linearFitXY(points) {
    const n = points.length;
    if (n < 2) return { slope: 0, intercept: points[0]?.y || 0 };
    let sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    for (const p of points) {
        sum_x += p.x;
        sum_y += p.y;
        sum_xy += p.x * p.y;
        sum_xx += p.x * p.x;
    }
    const slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    const intercept = (sum_y - slope * sum_x) / n;
    return { slope, intercept };
}
function drawChangepoints(changepoints, color) {
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    changepoints.forEach(cp_idx => {
        if (cp_idx === 0 || cp_idx >= trajectories[0].length) return;
        const point = trajectories[0][cp_idx]; // Use reference trajectory for x position
        ctx.beginPath();
        ctx.moveTo(point.x, 0);
        ctx.lineTo(point.x, canvas.height);
        ctx.stroke();
    })
}


// --- Main Execution Logic ---

function handleRun() {
    if (trajectories.length === 0 || trajectories[0].length < 2) {
        infoDisplay.textContent = "Please draw at least one trajectory before running.";
        return;
    }

    infoDisplay.textContent = "Running...";
    redrawAll(); // Clear previous results and show just the trajectories

    // Use a timeout to allow the "Running..." message to render
    setTimeout(() => {
        const factor = parseInt(downsampleSlider.value, 10);
        let startTime = performance.now();

        const downsampledTrajs = trajectories.map(t => downsample(t, factor));
        const alignedTrajs = alignTrajectories(downsampledTrajs);
        const { tubeMin, tubeMax } = calculateTube(alignedTrajs);

        let alignmentTime = performance.now() - startTime;
        drawTube(tubeMin, tubeMax);

        // Run SEGDP
        startTime = performance.now();
        const segdp_cps = runSEGDP(tubeMin, tubeMax);
        let segdpTime = performance.now() - startTime;
        drawSegmentation(segdp_cps, tubeMin, tubeMax, '#63B3ED', '#4299E1', false); // Blues

        // Run FastSEGDP
        startTime = performance.now();
        const fastsegdp_cps = runFastSEGDP(tubeMin, tubeMax);
        let fastsegdpTime = performance.now() - startTime;
        drawSegmentation(fastsegdp_cps, tubeMin, tubeMax, '#68D391', '#48BB78', true); // Greens

        infoDisplay.textContent = `Segmentation Complete:
- Alignment Time: ${alignmentTime.toFixed(1)} ms
- SEGDP Found: ${segdp_cps.length - 1} segments (${segdpTime.toFixed(1)} ms)
- FastSEGDP Found: ${fastsegdp_cps.length - 1} segments (${fastsegdpTime.toFixed(1)} ms)
Blue/Light Blue: SEGDP | Green/Light Green: FastSEGDP`;

    }, 50);
}

function handleClear() {
    trajectories = [];
    currentTrajectory = [];
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    infoDisplay.textContent = "Draw a trajectory on the canvas and click \"Run Segmentation\".";
}


// --- Event Listeners ---
document.addEventListener('DOMContentLoaded', () => {
    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);

    canvas.addEventListener('mousedown', startDrawing);
    canvas.addEventListener('mousemove', draw);
    canvas.addEventListener('mouseup', stopDrawing);
    canvas.addEventListener('mouseout', stopDrawing);

    // For touch devices
    canvas.addEventListener('touchstart', (e) => { e.preventDefault(); startDrawing(e.touches[0]); });
    canvas.addEventListener('touchmove', (e) => { e.preventDefault(); draw(e.touches[0]); });
    canvas.addEventListener('touchend', stopDrawing);

    runBtn.addEventListener('click', handleRun);
    clearBtn.addEventListener('click', handleClear);
    downsampleSlider.addEventListener('input', () => {
        downsampleValueSpan.textContent = downsampleSlider.value;
    });
});