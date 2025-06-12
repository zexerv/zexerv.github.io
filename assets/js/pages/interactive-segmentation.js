// --- DOM Elements ---
const canvas = document.getElementById('segmentation-canvas');
const ctx = canvas.getContext('2d');
const dtwCanvas = document.getElementById('dtw-canvas');
const dtwCtx = dtwCanvas.getContext('2d');

const alignBtn = document.getElementById('align-btn');
const segmentBtn = document.getElementById('segment-btn');
const clearBtn = document.getElementById('clear-canvas-btn');
const infoDisplay = document.getElementById('info-display');

const downsampleSlider = document.getElementById('downsample-slider');
const downsampleValueSpan = document.getElementById('downsample-value');
const lambdaSegdpSlider = document.getElementById('lambda-segdp-slider');
const lambdaSegdpValueSpan = document.getElementById('lambda-segdp-value');
const lambdaPeltSlider = document.getElementById('lambda-pelt-slider');
const lambdaPeltValueSpan = document.getElementById('lambda-pelt-value');

// --- Global State ---
let isDrawing = false;
let rawTrajectories = [];
let currentTrajectory = [];
let alignedTrajectories = [];
let tube = { tubeMin: null, tubeMax: null };

// --- Canvas & Drawing Setup ---
function resizeCanvas(canvasEl, context) {
    const dpr = window.devicePixelRatio || 1;
    const rect = canvasEl.getBoundingClientRect();
    canvasEl.width = rect.width * dpr;
    canvasEl.height = rect.height * dpr;
    context.scale(dpr, dpr);
}

function getMousePos(evt) {
    const rect = canvas.getBoundingClientRect();
    return {
        x: (evt.clientX - rect.left),
        y: (evt.clientY - rect.top)
    };
}

function startDrawing(e) {
    isDrawing = true;
    currentTrajectory = [getMousePos(e)];
    rawTrajectories.push(currentTrajectory);
    ctx.beginPath();
    ctx.moveTo(currentTrajectory[0].x, currentTrajectory[0].y);
    ctx.strokeStyle = '#E2E8F0';
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
    segmentBtn.disabled = true; // Require re-alignment after drawing
}

// --- Algorithm Implementations ---

// Downsampling
function downsample(trajectory, factor) {
    if (factor <= 1 || trajectory.length < factor) return trajectory;
    const downsampled = [];
    for (let i = 0; i < trajectory.length; i += factor) {
        const chunk = trajectory.slice(i, Math.min(i + factor, trajectory.length));
        const avg = chunk.reduce((acc, p) => ({ x: acc.x + p.x, y: acc.y + p.y }), { x: 0, y: 0 });
        avg.x /= chunk.length;
        avg.y /= chunk.length;
        downsampled.push(avg);
    }
    return downsampled;
}

// Alignment (DTW)
function euclideanDistance(p1, p2) {
    return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
}

function dtw(refSeq, trajSeq) {
    const n = refSeq.length;
    const m = trajSeq.length;
    const costMatrix = Array(n).fill(null).map(() => Array(m).fill(Infinity));
    costMatrix[0][0] = euclideanDistance(refSeq[0], trajSeq[0]);

    for (let i = 1; i < n; i++) costMatrix[i][0] = costMatrix[i-1][0] + euclideanDistance(refSeq[i], trajSeq[0]);
    for (let j = 1; j < m; j++) costMatrix[0][j] = costMatrix[0][j-1] + euclideanDistance(refSeq[0], trajSeq[j]);

    for (let i = 1; i < n; i++) {
        for (let j = 1; j < m; j++) {
            const cost = euclideanDistance(refSeq[i], trajSeq[j]);
            costMatrix[i][j] = cost + Math.min(costMatrix[i-1][j], costMatrix[i][j-1], costMatrix[i-1][j-1]);
        }
    }
    return costMatrix; // Return the full matrix
}

function backtrackDtw(costMatrix) {
    let path = [];
    let i = costMatrix.length - 1;
    let j = costMatrix[0].length - 1;
    path.push([i, j]);
    while (i > 0 || j > 0) {
        if (i > 0 && j > 0) {
            const minPrev = Math.min(costMatrix[i-1][j], costMatrix[i][j-1], costMatrix[i-1][j-1]);
            if (minPrev === costMatrix[i-1][j-1]) { i--; j--; }
            else if (minPrev === costMatrix[i-1][j]) { i--; }
            else { j--; }
        } else if (i > 0) i--;
        else j--;
        path.push([i, j]);
    }
    return path.reverse();
}

function align(trajectories) {
    if (trajectories.length <= 1) return { aligned: trajectories, costMatrix: null };
    const lengths = trajectories.map(t => t.length);
    const refIndex = lengths.indexOf(Math.max(...lengths));
    const refTraj = trajectories[refIndex];
    const aligned = [refTraj];
    let primaryCostMatrix = null;

    for (let i = 0; i < trajectories.length; i++) {
        if (i === refIndex) continue;
        const currentTraj = trajectories[i];
        const costMatrix = dtw(refTraj, currentTraj);
        if (i === (refIndex > 0 ? 0 : 1)) primaryCostMatrix = costMatrix; // Save first comparison matrix
        const path = backtrackDtw(costMatrix);
        const warpedTraj = [];
        let lastTrajIndex = 0;
        const refToTrajMap = {};
        path.forEach(([refI, trajI]) => { if (!(refI in refToTrajMap)) refToTrajMap[refI] = trajI; });
        for (let j = 0; j < refTraj.length; j++) {
             if (j in refToTrajMap) lastTrajIndex = refToTrajMap[j];
             warpedTraj.push(currentTraj[lastTrajIndex]);
        }
        aligned.push(warpedTraj);
    }
    return { aligned, costMatrix: primaryCostMatrix };
}


// Tube Calculation
function calculateTube(trajs) {
    const n = trajs[0].length;
    const tMin = Array(n).fill(null).map(() => ({x: Infinity, y: Infinity}));
    const tMax = Array(n).fill(null).map(() => ({x: -Infinity, y: -Infinity}));
    for(let i = 0; i < n; i++) {
        for(let j = 0; j < trajs.length; j++) {
            const point = trajs[j][i];
            if (!point) continue;
            tMin[i].x = Math.min(tMin[i].x, point.x);
            tMin[i].y = Math.min(tMin[i].y, point.y);
            tMax[i].x = Math.max(tMax[i].x, point.x);
            tMax[i].y = Math.max(tMax[i].y, point.y);
        }
    }
    return { tubeMin: tMin, tubeMax: tMax };
}

// Segmentation Algorithms (SEGDP & FastSEGDP/PELT - logic remains the same)
function getSegmentCostSEGDP(start, end, tubeMin, tubeMax) {
    const segmentLen = end - start + 1;
    if (segmentLen <= 1) return 0;
    let cost = 0;
    for (const dim of ['x', 'y']) {
        const min_slope = (tubeMin[end][dim] - tubeMin[start][dim]) / (segmentLen - 1);
        const max_slope = (tubeMax[end][dim] - tubeMax[start][dim]) / (segmentLen - 1);
        for (let i = 0; i < segmentLen; i++) {
            cost += Math.pow(tubeMin[start + i][dim] - (tubeMin[start][dim] + min_slope * i), 2);
            cost += Math.pow(tubeMax[start + i][dim] - (tubeMax[start][dim] + max_slope * i), 2);
        }
    }
    return cost;
}
// REPLACE the old runSEGDP function with this corrected one.

function runSEGDP(tubeMin, tubeMax, lambda) {
    const N = tubeMin.length;
    if (N === 0) return [];
    const MAX_SEGMENTS = Math.min(N, 30); // Keep a reasonable limit

    // Use a cache instead of pre-computing all costs to avoid freezing
    const costCache = {};
    const getCost = (start, end) => {
        const key = `${start}-${end}`;
        if (key in costCache) return costCache[key];
        const cost = getSegmentCostSEGDP(start, end, tubeMin, tubeMax);
        costCache[key] = cost;
        return cost;
    };

    const dp = Array.from({length: N}, () => Array(MAX_SEGMENTS + 1).fill(Infinity));
    const bp = Array.from({length: N}, () => Array(MAX_SEGMENTS + 1).fill(-1));

    // Initialization for 1 segment
    for (let t = 0; t < N; t++) {
        dp[t][1] = getCost(0, t);
        bp[t][1] = 0;
    }

    // Main DP loop
    for (let m = 2; m <= MAX_SEGMENTS; m++) {
        for (let t = 0; t < N; t++) {
            for (let j = 0; j < t; j++) {
                // Fetch cost from cache or compute it
                const currentCost = dp[j][m-1] + getCost(j + 1, t);
                if (currentCost < dp[t][m]) {
                    dp[t][m] = currentCost;
                    bp[t][m] = j + 1;
                }
            }
        }
    }

    // Find optimal number of segments using lambda
    let bestNumSegments = 1;
    let minTotalCost = dp[N-1][1] + lambda;
    for (let m = 2; m <= MAX_SEGMENTS; m++) {
        const totalCost = dp[N-1][m] + (lambda * m);
        if (totalCost < minTotalCost) {
            minTotalCost = totalCost;
            bestNumSegments = m;
        }
    }

    // Backtrack to find changepoints
    const changepoints = [N];
    let current_t = N - 1;
    let current_m = bestNumSegments;
    while (current_m > 0 && current_t >= 0) {
        let startOfSeg = bp[current_t][current_m];
        if (startOfSeg === -1) break;
        changepoints.push(startOfSeg);
        current_t = startOfSeg - 1;
        current_m--;
    }
    
    return changepoints.sort((a,b) => a-b).filter((v,i,a) => a.indexOf(v)===i);
}

function linearFit(points) {
    const n = points.length; if (n === 0) return {error: 0};
    let sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    for (let i = 0; i < n; i++) { sum_x += i; sum_y += points[i]; sum_xy += i * points[i]; sum_xx += i * i; }
    const slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    const intercept = (sum_y - slope * sum_x) / n;
    let error = 0;
    for (let i = 0; i < n; i++) error += Math.pow(points[i] - (slope * i + intercept), 2);
    return { error };
}

function getSegmentCostPELT(start, end, tubeMin, tubeMax) {
    const cost = linearFit(tubeMin.slice(start, end + 1).map(p => p.x)).error +
                 linearFit(tubeMin.slice(start, end + 1).map(p => p.y)).error +
                 linearFit(tubeMax.slice(start, end + 1).map(p => p.x)).error +
                 linearFit(tubeMax.slice(start, end + 1).map(p => p.y)).error;
    return cost;
}

function runFastSEGDP(tubeMin, tubeMax, lambda) {
    const N = tubeMin.length;
    const F = Array(N + 1).fill(Infinity);
    const P = Array(N + 1).fill(0);
    F[0] = -lambda;
    let R = [0];
    const costCache = {};
    const getCost = (s, e) => {
        const key = `${s}-${e}`; if (key in costCache) return costCache[key];
        return costCache[key] = getSegmentCostPELT(s, e, tubeMin, tubeMax);
    };
    for (let t = 1; t <= N; t++) {
        let best_tau = 0, min_cost = Infinity;
        for (const tau of R) {
            const cost = F[tau] + getCost(tau, t - 1) + lambda;
            if (cost < min_cost) { min_cost = cost; best_tau = tau; }
        }
        F[t] = min_cost; P[t] = best_tau;
        R = R.filter(tau => F[tau] + getCost(tau, t - 1) <= F[t]);
        R.push(t);
    }
    const changepoints = [N]; let t = N;
    while(t > 0) { let cp = P[t]; changepoints.push(cp); t = cp; }
    return changepoints.sort((a,b) => a-b).filter((v,i,a) => a.indexOf(v)===i);
}

// --- Visualization ---

function clearCanvas(ctxToClear) {
    ctxToClear.clearRect(0, 0, ctxToClear.canvas.width, ctxToClear.canvas.height);
}

function drawTrajectories(ctx, trajs, color, lineWidth) {
    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    trajs.forEach(traj => {
        if (traj.length < 2) return;
        ctx.beginPath();
        ctx.moveTo(traj[0].x, traj[0].y);
        for (let i = 1; i < traj.length; i++) ctx.lineTo(traj[i].x, traj[i].y);
        ctx.stroke();
    });
}

function drawTube(ctx, tMin, tMax) {
    ctx.fillStyle = 'rgba(100, 116, 139, 0.3)';
    ctx.beginPath();
    ctx.moveTo(tMax[0].x, tMax[0].y);
    for (let i = 1; i < tMax.length; i++) ctx.lineTo(tMax[i].x, tMax[i].y);
    for (let i = tMin.length - 1; i >= 0; i--) ctx.lineTo(tMin[i].x, tMin[i].y);
    ctx.closePath();
    ctx.fill();
}

function drawCostMatrix(ctx, matrix) {
    if (!matrix) return;
    const n = matrix.length;
    const m = matrix[0].length;
    const w = ctx.canvas.width / m;
    const h = ctx.canvas.height / n;
    const maxCost = matrix[n-1][m-1];
    for (let i = 0; i < n; i++) {
        for (let j = 0; j < m; j++) {
            const value = matrix[i][j] / maxCost;
            const colorVal = 255 - Math.floor(value * 255);
            ctx.fillStyle = `rgb(${colorVal}, ${colorVal}, ${colorVal})`;
            ctx.fillRect(j * w, i * h, w, h);
        }
    }
}

function drawSegmentation(ctx, changepoints, tMin, tMax) {
    // Corrected SEGDP visualization: continuous piecewise lines
    ctx.lineWidth = 3;
    // Min boundary
    ctx.strokeStyle = '#63B3ED'; // Blue
    ctx.beginPath();
    ctx.moveTo(tMin[changepoints[0]].x, tMin[changepoints[0]].y);
    for(let i=1; i < changepoints.length; i++) {
        const endIdx = changepoints[i] === tMin.length ? changepoints[i] - 1 : changepoints[i];
        ctx.lineTo(tMin[endIdx].x, tMin[endIdx].y);
    }
    ctx.stroke();
    // Max boundary
    ctx.strokeStyle = '#4299E1'; // Darker Blue
    ctx.beginPath();
    ctx.moveTo(tMax[changepoints[0]].x, tMax[changepoints[0]].y);
    for(let i=1; i < changepoints.length; i++) {
        const endIdx = changepoints[i] === tMax.length ? changepoints[i] - 1 : changepoints[i];
        ctx.lineTo(tMax[endIdx].x, tMax[endIdx].y);
    }
    ctx.stroke();
}

function linearFitXY(points) {
    const n = points.length; if (n < 2) return { slope: 0, intercept: points[0]?.y || 0 };
    let sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    for (const p of points) { sum_x += p.x; sum_y += p.y; sum_xy += p.x * p.y; sum_xx += p.x * p.x; }
    const slope = (n * sum_xx - sum_x * sum_x) === 0 ? 0 : (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    const intercept = (sum_y - slope * sum_x) / n;
    return { slope, intercept };
}


function drawVerticalChangepoints(ctx, changepoints, trajs, color, style = 'solid') {
    if (trajs.length === 0) return;
    const refTraj = trajs[0];
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    if (style === 'dashed') ctx.setLineDash([5, 5]);
    else if (style === 'dotted') ctx.setLineDash([2, 3]);

    changepoints.forEach(cp_idx => {
        if (cp_idx === 0 || cp_idx >= refTraj.length) return;
        const point = refTraj[cp_idx];
        ctx.beginPath();
        ctx.moveTo(point.x, 0);
        ctx.lineTo(point.x, ctx.canvas.height);
        ctx.stroke();
    });
    ctx.setLineDash([]); // Reset line dash
}

function drawLegend(ctx) {
    const legend = [
        { text: 'SEGDP Approx.', color: '#63B3ED' },
        { text: 'SEGDP CPs', color: '#63B3ED', style: 'dotted' },
        { text: 'FastSEGDP CPs', color: '#68D391', style: 'dashed' },
    ];
    ctx.font = '12px Roboto Mono';
    let xOffset = 10;
    legend.forEach(item => {
        ctx.fillStyle = item.color;
        ctx.strokeStyle = item.color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        if (item.style === 'dotted') ctx.setLineDash([2, 3]);
        else if (item.style === 'dashed') ctx.setLineDash([5, 5]);
        ctx.moveTo(xOffset, 15);
        ctx.lineTo(xOffset + 20, 15);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillText(item.text, xOffset + 25, 20);
        xOffset += ctx.measureText(item.text).width + 50;
    });
}


// --- Main Execution Logic & Event Handlers ---

function handleAlign() {
    if (rawTrajectories.length === 0 || rawTrajectories[0].length < 2) {
        infoDisplay.textContent = "Please draw at least one trajectory.";
        return;
    }
    infoDisplay.textContent = "Aligning...";
    clearCanvas(ctx);
    clearCanvas(dtwCtx);
    drawTrajectories(ctx, rawTrajectories, '#A0AEC0', 1.5); // Draw raw

    setTimeout(() => {
        const factor = parseInt(downsampleSlider.value, 10);
        const startTime = performance.now();
        const downsampled = rawTrajectories.map(t => downsample(t, factor));
        const { aligned, costMatrix } = align(downsampled);
        const alignmentTime = performance.now() - startTime;
        
        alignedTrajectories = aligned;
        tube = calculateTube(alignedTrajectories);
        
        clearCanvas(ctx);
        drawTrajectories(ctx, alignedTrajectories, '#FDBA74', 1); // Draw aligned (orange)
        drawTube(ctx, tube.tubeMin, tube.tubeMax);
        drawCostMatrix(dtwCtx, costMatrix);
        
        infoDisplay.textContent = `Alignment Complete: ${alignmentTime.toFixed(1)} ms.\nReady for segmentation.`;
        segmentBtn.disabled = false;
    }, 50);
}

function handleSegment() {
    if (alignedTrajectories.length === 0) {
        infoDisplay.textContent = "Please align trajectories first.";
        return;
    }
    infoDisplay.textContent = "Segmenting...";
    clearCanvas(ctx);
    drawTrajectories(ctx, alignedTrajectories, '#FDBA74', 1);
    drawTube(ctx, tube.tubeMin, tube.tubeMax);

    setTimeout(() => {
        const lambdaSEGDP = parseFloat(lambdaSegdpSlider.value);
        const lambdaPELT = parseFloat(lambdaPeltSlider.value);

        // Run SEGDP
        const startTimeSEGDP = performance.now();
        const segdp_cps = runSEGDP(tube.tubeMin, tube.tubeMax, lambdaSEGDP);
        const segdpTime = performance.now() - startTimeSEGDP;
        drawSegmentation(ctx, segdp_cps, tube.tubeMin, tube.tubeMax);
        drawVerticalChangepoints(ctx, segdp_cps, alignedTrajectories, '#63B3ED', 'dotted');

        // Run FastSEGDP
        const startTimePELT = performance.now();
        const fastsegdp_cps = runFastSEGDP(tube.tubeMin, tube.tubeMax, lambdaPELT);
        const peltTime = performance.now() - startTimePELT;
        // FastSEGDP viz is just vertical lines as requested
        drawVerticalChangepoints(ctx, fastsegdp_cps, alignedTrajectories, '#68D391', 'dashed');

        drawLegend(ctx);

        infoDisplay.textContent = `SEGDP: ${segdp_cps.length - 1} segments (${segdpTime.toFixed(1)} ms)\nFastSEGDP: ${fastsegdp_cps.length - 1} segments (${peltTime.toFixed(1)} ms)`;
    }, 50);
}


function handleClear() {
    rawTrajectories = [];
    alignedTrajectories = [];
    tube = { tubeMin: null, tubeMax: null };
    clearCanvas(ctx);
    clearCanvas(dtwCtx);
    segmentBtn.disabled = true;
    infoDisplay.textContent = 'Draw a trajectory and click "Align Trajectories".';
}

function setupEventListeners() {
    resizeCanvas(canvas, ctx);
    resizeCanvas(dtwCanvas, dtwCtx);
    window.addEventListener('resize', () => {
        resizeCanvas(canvas, ctx);
        resizeCanvas(dtwCanvas, dtwCtx);
        // Could add redraw logic here if needed
    });

    canvas.addEventListener('mousedown', startDrawing);
    canvas.addEventListener('mousemove', draw);
    canvas.addEventListener('mouseup', stopDrawing);
    canvas.addEventListener('mouseout', stopDrawing);
    canvas.addEventListener('touchstart', (e) => { e.preventDefault(); startDrawing(e.touches[0]); });
    canvas.addEventListener('touchmove', (e) => { e.preventDefault(); draw(e.touches[0]); });
    canvas.addEventListener('touchend', stopDrawing);

    alignBtn.addEventListener('click', handleAlign);
    segmentBtn.addEventListener('click', handleSegment);
    clearBtn.addEventListener('click', handleClear);
    
    downsampleSlider.addEventListener('input', () => downsampleValueSpan.textContent = downsampleSlider.value);
    lambdaSegdpSlider.addEventListener('input', () => lambdaSegdpValueSpan.textContent = lambdaSegdpSlider.value);
    lambdaPeltSlider.addEventListener('input', () => lambdaPeltValueSpan.textContent = `${(parseFloat(lambdaPeltSlider.value)/1000000).toFixed(1)}M`);
    // Trigger initial value display
    lambdaPeltValueSpan.textContent = `${(parseFloat(lambdaPeltSlider.value)/1000000).toFixed(1)}M`;
}

document.addEventListener('DOMContentLoaded', setupEventListeners);