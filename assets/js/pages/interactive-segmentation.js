document.addEventListener('DOMContentLoaded', () => {

    // --- DOM Element Selection ---
    const canvas = document.getElementById('segmentation-canvas');
    if (!canvas) { console.error("Could not find segmentation-canvas"); return; }
    const ctx = canvas.getContext('2d');

    const dtwCanvas = document.getElementById('dtw-canvas');
    if (!dtwCanvas) { console.error("Could not find dtw-canvas"); return; }
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
    let resampledTrajectories = [];
    let alignedTrajectories = [];
    let tube = { tubeMin: null, tubeMax: null };

    // --- Core Action Handlers ---
    const handleClear = () => {
        rawTrajectories = [];
        resampledTrajectories = [];
        alignedTrajectories = [];
        tube = { tubeMin: null, tubeMax: null };
        clearCanvas(ctx);
        clearCanvas(dtwCtx);
        segmentBtn.disabled = true;
        infoDisplay.textContent = 'Draw one or more trajectories and click "Align".';
    };

    const handleAlign = () => {
        if (rawTrajectories.length < 2) {
            infoDisplay.textContent = "Please draw at least two trajectories to align.";
            return;
        }
        infoDisplay.textContent = "Resampling & Aligning...";
        segmentBtn.disabled = true;

        setTimeout(() => {
            resampledTrajectories = rawTrajectories.map(t => resampleTrajectoryByX(t)).filter(t => t.length > 1);

            if (resampledTrajectories.length < 2) {
                infoDisplay.textContent = "Could not process drawings. Please draw wider, more distinct trajectories.";
                return;
            }

            const factor = parseInt(downsampleSlider.value, 10);
            const downsampled = resampledTrajectories.map(t => downsample(t, factor));

            // Fix #1 & #2 Cont.: The `align` function now returns both aligned trajectories and the data needed for visualization.
            const { aligned, alignmentData } = align(downsampled);

            alignedTrajectories = aligned;

            if (aligned.length > 0 && aligned.flat().length > 0) {
                 tube = calculateTube(alignedTrajectories);
                 segmentBtn.disabled = false;
                 infoDisplay.textContent = `Alignment complete. Found ${alignmentData.length} DTW comparisons. Ready for segmentation.`;
            } else {
                 infoDisplay.textContent = `Alignment failed. Please clear and try again.`;
            }

            drawState();
            // Pass all alignment data to be drawn.
            drawCostMatrices(dtwCtx, alignmentData);
        }, 10);
    };

    const handleSegment = () => {
        if (alignedTrajectories.length === 0) {
            infoDisplay.textContent = "Please align trajectories first.";
            return;
        }
        infoDisplay.textContent = "Segmenting...";

        setTimeout(() => {
            // The tube is already calculated from the *aligned* trajectories in handleAlign.
            // We just need to normalize it for the algorithms.
            const normalizedTube = normalizeTube(tube);
            const lambdaSEGDP = getLogValue(parseFloat(lambdaSegdpSlider.value), 0.01, 100);
            const lambdaPELT = getLogValue(parseFloat(lambdaPeltSlider.value), 0.01, 100);

            const startTimeSEGDP = performance.now();
            const segdp_cps = runSEGDP(normalizedTube.tubeMin, normalizedTube.tubeMax, lambdaSEGDP);
            const segdpTime = performance.now() - startTimeSEGDP;

            const startTimePELT = performance.now();
            const fastsegdp_cps = runFastSEGDP(normalizedTube.tubeMin, normalizedTube.tubeMax, lambdaPELT);
            const peltTime = performance.now() - startTimePELT;

            // The tube passed to drawState is the one based on aligned data, which is correct.
            drawState({ segdp_cps, fastsegdp_cps });

            infoDisplay.textContent = `SEGDP: ${segdp_cps.length - 1} segments (${segdpTime.toFixed(1)} ms)\nFastSEGDP: ${fastsegdp_cps.length - 1} segments (${peltTime.toFixed(1)} ms)`;
        }, 10);
    };

    // --- Drawing & Visualization ---
    const drawState = (segmentations = {}) => {
        clearCanvas(ctx);

        // The core logic is to decide WHAT to draw based on the state.
        // If we have aligned trajectories, we show them. Otherwise, show the raw drawings.
        if (alignedTrajectories.length > 0) {
            const vizTrajectories = spatiallyNormalizeForViz(alignedTrajectories);
            drawTrajectories(ctx, vizTrajectories, '#475569', 1.0);

            // The tube is calculated on aligned data. We need to normalize it for visualization
            // in the same way the trajectories were, so they match up.
            if (tube.tubeMin) {
                const vizTube = {
                    tubeMin: spatiallyNormalizeForViz([tube.tubeMin])[0],
                    tubeMax: spatiallyNormalizeForViz([tube.tubeMax])[0]
                };
                drawTube(ctx, vizTube.tubeMin, vizTube.tubeMax);
            }

        } else {
            drawTrajectories(ctx, rawTrajectories, '#475569', 1.5);
        }

        if (segmentations.segdp_cps) {
            // Also normalize the tube for drawing segmentations
            const vizTube = {
                tubeMin: spatiallyNormalizeForViz([tube.tubeMin])[0],
                tubeMax: spatiallyNormalizeForViz([tube.tubeMax])[0]
            };
            drawSegmentation(ctx, segmentations.segdp_cps, vizTube.tubeMin, vizTube.tubeMax, false);
            drawVerticalChangepoints(ctx, segmentations.segdp_cps, spatiallyNormalizeForViz(alignedTrajectories), '#63B3ED', 'dotted');
        }
        if (segmentations.fastsegdp_cps) {
            const vizTube = {
                tubeMin: spatiallyNormalizeForViz([tube.tubeMin])[0],
                tubeMax: spatiallyNormalizeForViz([tube.tubeMax])[0]
            };
            drawSegmentation(ctx, segmentations.fastsegdp_cps, vizTube.tubeMin, vizTube.tubeMax, true);
            drawVerticalChangepoints(ctx, segmentations.fastsegdp_cps, spatiallyNormalizeForViz(alignedTrajectories), '#68D391', 'dashed');
        }
        if (segmentations.segdp_cps || segmentations.fastsegdp_cps) {
            drawLegend(ctx);
        }
    };

    // --- Event Listeners Setup ---
    const setupEventListeners = () => {
        const startDrawing = (e) => {
            isDrawing = true;
            resampledTrajectories = [];
            alignedTrajectories = [];
            tube = { tubeMin: null, tubeMax: null };
            segmentBtn.disabled = true;
            clearCanvas(dtwCtx);
            rawTrajectories.push([getMousePos(e)]);
        };
        const moveDrawing = (e) => {
            if(isDrawing) {
                rawTrajectories[rawTrajectories.length - 1].push(getMousePos(e));
                drawState();
            }
        };
        const endDrawing = () => { isDrawing = false; };

        canvas.addEventListener('mousedown', startDrawing);
        canvas.addEventListener('mousemove', moveDrawing);
        canvas.addEventListener('mouseup', endDrawing);
        canvas.addEventListener('mouseout', endDrawing);

        canvas.addEventListener('touchstart', (e)=>{e.preventDefault();startDrawing(e.touches[0])});
        canvas.addEventListener('touchmove', (e)=>{e.preventDefault();moveDrawing(e.touches[0])});
        canvas.addEventListener('touchend', endDrawing);

        alignBtn.addEventListener('click', handleAlign);
        segmentBtn.addEventListener('click', handleSegment);
        clearBtn.addEventListener('click', handleClear);

        downsampleSlider.addEventListener('input', () => { downsampleValueSpan.textContent = downsampleSlider.value; });
        lambdaSegdpSlider.addEventListener('input', () => {
            const logValue = getLogValue(parseFloat(lambdaSegdpSlider.value), 0.01, 100);
            lambdaSegdpValueSpan.textContent = logValue.toFixed(4);
        });
        lambdaPeltSlider.addEventListener('input', () => {
            const logValue = getLogValue(parseFloat(lambdaPeltSlider.value), 0.01, 100);
            lambdaPeltValueSpan.textContent = logValue.toFixed(4);
        });
    };

    // --- Initial Call & Utility ---
    const resizeAllCanvases = () => {
        const dpr = window.devicePixelRatio || 1;
        [canvas, dtwCanvas].forEach(c => {
            if (!c) return;
            const rect = c.getBoundingClientRect();
            c.width = rect.width * dpr;
            c.height = rect.height * dpr;
            c.getContext('2d').scale(dpr, dpr);
        });
        drawState();
    };
    const getMousePos = (evt) => {
        const rect = canvas.getBoundingClientRect();
        return { x: evt.clientX - rect.left, y: evt.clientY - rect.top };
    };

    handleClear();
    resizeAllCanvases();
    setupEventListeners();

    downsampleValueSpan.textContent = downsampleSlider.value;
    lambdaSegdpValueSpan.textContent = getLogValue(parseFloat(lambdaSegdpSlider.value), 0.01, 100).toFixed(4);
    lambdaPeltValueSpan.textContent = getLogValue(parseFloat(lambdaPeltSlider.value), 0.01, 100).toFixed(4);
});


// --- ALGORITHMS & VISUALIZATION HELPERS ---

const clearCanvas = (ctx) => {
    if (ctx && ctx.canvas) {
        const dpr = window.devicePixelRatio || 1;
        ctx.clearRect(0, 0, ctx.canvas.width / dpr, ctx.canvas.height / dpr);
    }
};

const getLogValue = (pos, min, max) => Math.exp(Math.log(min) + (Math.log(max) - Math.log(min)) / 100 * pos);

const drawTrajectories = (ctx, trajs, color, lineWidth) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    trajs.forEach(traj => {
        if (traj.length < 2) return;
        ctx.beginPath();
        ctx.moveTo(traj[0].x, traj[0].y);
        for (let i = 1; i < traj.length; i++) {
            ctx.lineTo(traj[i].x, traj[i].y);
        }
        ctx.stroke();
    });
};

const drawTube = (ctx, tMin, tMax) => {
    if (!tMin || tMin.length === 0) return;
    ctx.fillStyle = 'rgba(100, 116, 139, 0.3)';
    ctx.beginPath();
    ctx.moveTo(tMax[0].x, tMax[0].y);
    for (let i = 1; i < tMax.length; i++) ctx.lineTo(tMax[i].x, tMax[i].y);
    for (let i = tMin.length - 1; i >= 0; i--) ctx.lineTo(tMin[i].x, tMin[i].y);
    ctx.closePath();
    ctx.fill();
};

const drawSegmentation = (ctx, cps, tMin, tMax, isFast) => {
    if (!cps || cps.length < 2 || !tMin || tMin.length === 0) return;

    for(let i = 0; i < cps.length - 1; i++) {
        const start = cps[i];
        let end = cps[i+1];
        if (isFast) end--;
        if (end < start || end >= tMin.length) continue;

        if (isFast) {
            const pointsMin = tMin.slice(start, end + 1);
            const pointsMax = tMax.slice(start, end + 1);
            if (pointsMin.length < 2 || pointsMax.length < 2) continue;

            const fitMin = linearFitXY(pointsMin);
            const fitMax = linearFitXY(pointsMax);

            const startX = pointsMin[0].x;
            const endX = pointsMin[pointsMin.length - 1].x;

            ctx.strokeStyle = '#68D391';
            ctx.lineWidth = 2.5;
            ctx.beginPath();
            ctx.moveTo(startX, fitMin.slope * startX + fitMin.intercept);
            ctx.lineTo(endX, fitMin.slope * endX + fitMin.intercept);
            ctx.stroke();

            ctx.strokeStyle = '#48BB78';
            ctx.beginPath();
            ctx.moveTo(startX, fitMax.slope * startX + fitMax.intercept);
            ctx.lineTo(endX, fitMax.slope * endX + fitMax.intercept);
            ctx.stroke();

        } else {
            ctx.lineWidth = 3;
            ctx.strokeStyle = '#63B3ED';
            ctx.beginPath();
            if(tMin[start]) ctx.moveTo(tMin[start].x, tMin[start].y);
            if(tMin[end]) ctx.lineTo(tMin[end].x, tMin[end].y);
            ctx.stroke();

            ctx.strokeStyle = '#4299E1';
            ctx.beginPath();
            if(tMax[start]) ctx.moveTo(tMax[start].x, tMax[start].y);
            if(tMax[end]) ctx.lineTo(tMax[end].x, tMax[end].y);
            ctx.stroke();
        }
    }
};

const drawVerticalChangepoints = (ctx, cps, trajs, color, style) => {
    if (!trajs || trajs.length === 0 || !cps) return;
    const refTraj = trajs[0];
    if(refTraj.length === 0) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    if (style === 'dashed') ctx.setLineDash([5, 5]);
    else if (style === 'dotted') ctx.setLineDash([2, 3]);

    cps.forEach(cp_idx => {
        if (cp_idx > 0 && cp_idx < refTraj.length) {
            const point = refTraj[cp_idx];
            ctx.beginPath();
            ctx.moveTo(point.x, 0);
            ctx.lineTo(point.x, ctx.canvas.getBoundingClientRect().height);
            ctx.stroke();
        }
    });
    ctx.setLineDash([]);
};

const drawLegend = (ctx) => {
    const legend = [
        { text: 'SEGDP Approx.', color: '#63B3ED', style: 'solid' },
        { text: 'FastSEGDP Approx.', color: '#68D391', style: 'solid'},
        { text: 'SEGDP CPs', color: '#63B3ED', style: 'dotted' },
        { text: 'FastSEGDP CPs', color: '#68D391', style: 'dashed' },
    ];
    ctx.save();
    ctx.font = '12px Roboto Mono';
    ctx.textBaseline = 'middle';
    let xOffset = 10;
    legend.forEach(item => {
        ctx.strokeStyle = item.color;
        ctx.fillStyle = item.color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        if (item.style === 'dotted') ctx.setLineDash([2, 3]);
        else if (item.style === 'dashed') ctx.setLineDash([5, 5]);
        else ctx.setLineDash([]);
        ctx.moveTo(xOffset, 15);
        ctx.lineTo(xOffset + 20, 15);
        ctx.stroke();
        ctx.fillText(item.text, xOffset + 25, 15);
        xOffset += ctx.measureText(item.text).width + 50;
    });
    ctx.restore();
};

const drawCostMatrices = (ctx, alignmentData) => {
    // Fix #1 & #3: This function now receives the full alignment data and draws all matrices with their paths
    clearCanvas(ctx);
    if (!alignmentData || alignmentData.length === 0) return;

    const canvasWidth = ctx.canvas.getBoundingClientRect().width;
    const canvasHeight = ctx.canvas.getBoundingClientRect().height;
    const totalMatrices = alignmentData.length;
    const matrixHeight = canvasHeight / totalMatrices;

    alignmentData.forEach(({ matrix, path }, index) => {
        if (!matrix || matrix.length === 0) return;
        const n = matrix.length; const m = matrix[0].length;
        const w = canvasWidth / m; const h = matrixHeight / n;
        const yOffset = index * matrixHeight;
        const maxCost = matrix[n-1][m-1];
        if (maxCost === 0) return;

        // Draw Heatmap
        for (let i = 0; i < n; i++) for (let j = 0; j < m; j++) {
            const value = matrix[i][j] / maxCost;
            const colorVal = Math.floor((1 - value) * 200) + 55;
            ctx.fillStyle = `rgb(${colorVal}, ${colorVal}, ${colorVal})`;
            // Correct y-position calculation to flip the matrix visually
            const yPos = yOffset + (matrixHeight - (i + 1) * h);
            ctx.fillRect(j * w, yPos, w, h);
        }

        // Draw Warping Path
        ctx.strokeStyle = '#FDBA74'; // Orange for the path
        ctx.lineWidth = 2;
        ctx.beginPath();
        // Correctly map path coordinates to the flipped canvas coordinates
        const startY = yOffset + (matrixHeight - path[0][0] * h - h / 2);
        ctx.moveTo(path[0][1] * w + w / 2, startY);
        for(let k = 1; k < path.length; k++){
            const yPos = yOffset + (matrixHeight - path[k][0] * h - h / 2);
            ctx.lineTo(path[k][1] * w + w / 2, yPos);
        }
        ctx.stroke();
    });
};


const downsample = (traj, factor) => {
    if (factor <= 1 || traj.length < factor) return traj;
    const downsampled = [];
    for (let i = 0; i < traj.length; i += factor) {
        const chunk = traj.slice(i, Math.min(i + factor, traj.length));
        const avg = chunk.reduce((acc, p) => ({ x: acc.x + p.x, y: acc.y + p.y }), { x: 0, y: 0 });
        avg.x /= chunk.length; avg.y /= chunk.length;
        downsampled.push(avg);
    }
    return downsampled;
};

const resampleTrajectoryByX = (rawTraj) => {
    if (rawTraj.length < 2) return [];
    const sortedTraj = [...rawTraj].sort((a,b) => a.x - b.x);
    const resampled = [];
    const startX = Math.ceil(sortedTraj[0].x);
    const endX = Math.floor(sortedTraj[sortedTraj.length - 1].x);
    if (startX >= endX) return [];

    let rawIndex = 0;
    for (let x = startX; x <= endX; x++) {
        while(rawIndex + 1 < sortedTraj.length && sortedTraj[rawIndex+1].x < x) {
            rawIndex++;
        }
        const p1 = sortedTraj[rawIndex];
        const p2 = sortedTraj[rawIndex + 1];

        if (!p1 || !p2) continue;

        const x1 = p1.x, y1 = p1.y;
        const x2 = p2.x, y2 = p2.y;

        if (x2 - x1 === 0) {
            if(x === x1) resampled.push({ x, y: y1 });
        } else {
            const t = (x - x1) / (x2 - x1);
            const y = y1 + t * (y2 - y1);
            resampled.push({ x, y });
        }
    }
    return resampled;
};

const euclideanDistance = (p1, p2) => Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));

const dtw = (refSeq, trajSeq) => {
    const n = refSeq.length; const m = trajSeq.length;
    if (n === 0 || m === 0) return [[]];
    const costMatrix = Array.from({length: n}, () => Array(m).fill(Infinity));
    costMatrix[0][0] = euclideanDistance(refSeq[0], trajSeq[0]);
    for (let i = 1; i < n; i++) costMatrix[i][0] = costMatrix[i-1][0] + euclideanDistance(refSeq[i], trajSeq[0]);
    for (let j = 1; j < m; j++) costMatrix[0][j] = costMatrix[0][j-1] + euclideanDistance(refSeq[0], trajSeq[j]);
    for (let i = 1; i < n; i++) for (let j = 1; j < m; j++) {
        costMatrix[i][j] = euclideanDistance(refSeq[i], trajSeq[j]) + Math.min(costMatrix[i-1][j], costMatrix[i][j-1], costMatrix[i-1][j-1]);
    }
    return costMatrix;
};

const backtrackDtw = (costMatrix) => {
    if (!costMatrix || costMatrix.length === 0 || costMatrix[0].length === 0) return [];
    let path = [];
    let i = costMatrix.length - 1; let j = costMatrix[0].length - 1;
    path.push([i, j]);
    while (i > 0 || j > 0) {
        if (i > 0 && j > 0) {
            const minPrev = Math.min(costMatrix[i-1][j], costMatrix[i][j-1], costMatrix[i-1][j-1]);
            if (minPrev === costMatrix[i-1][j-1]) { i--; j--; }
            else if (minPrev === costMatrix[i-1][j]) { i--; }
            else { j--; }
        } else if (i > 0) i--; else j--;
        path.push([i, j]);
    }
    return path.reverse();
};

function align(trajectories) {
    // Fix #1 & #2: This function now correctly calculates and returns all N-1 alignment results
    if (trajectories.length <= 1) return { aligned: trajectories, alignmentData: [] };
    const refIndex = trajectories.map(t => t.length).indexOf(Math.max(...trajectories.map(t => t.length)));
    const refTraj = trajectories[refIndex];
    if (!refTraj || refTraj.length === 0) return { aligned: [], alignmentData: [] };

    const aligned = [refTraj];
    const alignmentData = []; // To store {matrix, path} for each alignment

    for (let i = 0; i < trajectories.length; i++) {
        if (i === refIndex) continue;
        const trajToAlign = trajectories[i];
        if (trajToAlign.length === 0) continue;

        const costMatrix = dtw(refTraj, trajToAlign);
        const path = backtrackDtw(costMatrix);
        alignmentData.push({matrix: costMatrix, path: path}); // Store result for visualization

        if (path.length === 0) continue;

        const warpedTraj = new Array(refTraj.length);
        let pathIdx = 0;
        for (let refIdx = 0; refIdx < refTraj.length; refIdx++) {
            while (pathIdx + 1 < path.length && path[pathIdx + 1][0] <= refIdx) {
                pathIdx++;
            }
            const mappedTrajIndex = path[pathIdx][1];
            warpedTraj[refIdx] = trajToAlign[mappedTrajIndex] || trajToAlign[trajToAlign.length - 1];
        }
        aligned.push(warpedTraj);
    }
    return { aligned, alignmentData };
}


const calculateTube = (trajs) => {
    if (trajs.length === 0 || trajs.some(t => !t || t.length === 0) || trajs[0].length === 0) return {tubeMin: [], tubeMax: []};
    const n = trajs[0].length;
    const tMin = Array.from({length:n},()=>({x:Infinity,y:Infinity}));
    const tMax = Array.from({length:n},()=>({x:-Infinity,y:-Infinity}));
    for(let i=0;i<n;i++) for(let j=0;j<trajs.length;j++) {
        if (!trajs[j] || !trajs[j][i]) continue;
        const p=trajs[j][i];
        tMin[i].x=Math.min(tMin[i].x,p.x); tMin[i].y=Math.min(tMin[i].y,p.y);
        tMax[i].x=Math.max(tMax[i].x,p.x); tMax[i].y=Math.max(tMax[i].y,p.y);
    }
    // This is a sanity check. If a point in the tube wasn't updated, copy from the previous valid point.
    for (let i = 1; i < n; i++) {
        if (tMin[i].x === Infinity) tMin[i] = { ...tMin[i-1] };
        if (tMax[i].x === -Infinity) tMax[i] = { ...tMax[i-1] };
    }
    return {tubeMin:tMin,tubeMax:tMax};
};

const normalizeTube = (tube) => {
    if (!tube || !tube.tubeMin || tube.tubeMin.length === 0) return tube;
    const allPoints = [...tube.tubeMin, ...tube.tubeMax];
    const minX = Math.min(...allPoints.map(p => p.x)); const maxX = Math.max(...allPoints.map(p => p.x));
    const minY = Math.min(...allPoints.map(p => p.y)); const maxY = Math.max(...allPoints.map(p => p.y));
    const rangeX = maxX - minX; const rangeY = maxY - minY;
    if (rangeX === 0 || rangeY === 0) return tube;
    const normalize = (points) => points.map(p => ({ x: (p.x - minX) / rangeX, y: (p.y - minY) / rangeY }));
    return { tubeMin: normalize(tube.tubeMin), tubeMax: normalize(tube.tubeMax) };
};

const getSegmentCostSEGDP = (start,end,tMin,tMax) => {
    const len=end-start+1; if(len<=1)return 0; let cost=0;
    for(const dim of ['x','y']){
        const min_s= tMin[end] && tMin[start] ? (tMin[end][dim]-tMin[start][dim])/(len-1) : 0;
        const max_s= tMax[end] && tMax[start] ? (tMax[end][dim]-tMax[start][dim])/(len-1) : 0;
        for(let i=0;i<len;i++){
            if(tMin[start+i]) cost+=Math.pow(tMin[start+i][dim]-(tMin[start][dim]+min_s*i),2);
            if(tMax[start+i]) cost+=Math.pow(tMax[start+i][dim]-(tMax[start][dim]+max_s*i),2);
        }
    }
    return cost;
};

function runSEGDP(tubeMin, tubeMax, lambda) {
    const N = tubeMin.length; if(N===0) return [];
    const MAX_SEGMENTS=Math.min(N,30);
    const costCache={}; const getCost=(s,e)=>{const k=`${s}-${e}`; if(k in costCache)return costCache[k]; return costCache[k]=getSegmentCostSEGDP(s,e,tubeMin,tubeMax);};
    const dp=Array.from({length:N},()=>Array(MAX_SEGMENTS+1).fill(Infinity));
    const bp=Array.from({length:N},()=>Array(MAX_SEGMENTS+1).fill(-1));
    for(let t=0;t<N;t++){dp[t][1]=getCost(0,t);bp[t][1]=0;}
    for(let m=2;m<=MAX_SEGMENTS;m++) for(let t=0;t<N;t++) for(let j=0;j<t;j++){
        const cost=dp[j][m-1]+getCost(j+1,t); if(cost<dp[t][m]){dp[t][m]=cost;bp[t][m]=j+1;}
    }
    let bestM=1,minCost=dp[N-1][1]+lambda;
    for(let m=2;m<=MAX_SEGMENTS;m++){const cost=dp[N-1][m]+(lambda*m); if(cost<minCost){minCost=cost;bestM=m;}}

    // Fix #4: Correct backtracking to ensure the last point is always included.
    const cps = [];
    if (N > 0) {
        cps.push(N);
        let t = N - 1;
        let m = bestM;
        while (m > 1 && t > 0) {
            let prev_t = bp[t][m];
            if (prev_t === -1 || prev_t >= t) break;
            cps.unshift(prev_t);
            t = prev_t - 1;
            m--;
        }
        cps.unshift(0);
    }
    return [...new Set(cps)].sort((a,b)=>a-b);
}

const linearFitXY = (points) => {
    const n = points.length; if (n < 2) return { slope: 0, intercept: points[0]?.y || 0 };
    let sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    for (const p of points) { sum_x += p.x; sum_y += p.y; sum_xy += p.x * p.y; sum_xx += p.x * p.x; }
    const den = (n * sum_xx - sum_x * sum_x);
    if(den === 0) return { slope: 0, intercept: points[0]?.y || 0 };
    const slope = (n * sum_xy - sum_x * sum_y) / den;
    const intercept = (sum_y - slope * sum_x) / n;
    return { slope, intercept };
};

const getSegmentCostPELT = (start, end, tMin, tMax) => {
    let cost = 0;
    const pointsMin = tMin.slice(start, end + 1);
    const pointsMax = tMax.slice(start, end + 1);
    if (pointsMin.length < 2) return 0;

    for(const dim of ['x', 'y']) {
        const fitMin = linearFitXY(pointsMin.map(p => ({ x: p.x, y: p[dim] })));
        const fitMax = linearFitXY(pointsMax.map(p => ({ x: p.x, y: p[dim] })));
        for(let i=0; i<pointsMin.length; i++) {
            cost += Math.pow(pointsMin[i][dim] - (fitMin.slope * pointsMin[i].x + fitMin.intercept), 2);
            cost += Math.pow(pointsMax[i][dim] - (fitMax.slope * pointsMax[i].x + fitMax.intercept), 2);
        }
    }
    return cost;
};


const spatiallyNormalizeForViz = (trajectories) => {
    if (trajectories.length < 2) return trajectories;

    // Find a valid reference trajectory
    const refTraj = trajectories.find(t => t && t.length > 0);
    if (!refTraj) return trajectories; // Return if no valid trajectories

    const refMinX = refTraj[0].x;
    const refMaxX = refTraj[refTraj.length - 1].x;
    const refRangeX = refMaxX - refMinX;
    if(refRangeX === 0) return trajectories;

    return trajectories.map((traj) => {
        if (!traj || traj.length === 0) return []; // Handle empty trajectories
        const trajMinX = traj[0].x;
        const trajMaxX = traj[traj.length - 1].x;
        const trajRangeX = trajMaxX - trajMinX;
        if (trajRangeX === 0) return traj.map(p => ({...p, x: refMinX}));

        return traj.map(p => ({
            x: refMinX + ((p.x - trajMinX) * refRangeX / trajRangeX),
            y: p.y
        }));
    });
};

function runFastSEGDP(tubeMin, tubeMax, lambda) {
    const N = tubeMin.length; if (N === 0) return [];
    const costCache = {};
    const getCost = (start, end) => {
        if (start > end) return 0;
        const key = `${start}-${end}`;
        if (key in costCache) return costCache[key];
        // Using a more robust cost function for PELT
        return costCache[key] = getSegmentCostSEGDP(start, end, tubeMin, tubeMax);
    };
    const F = Array(N + 1).fill(Infinity);
    const P = Array(N + 1).fill(0);
    F[0] = -lambda;
    let R = [0];
    for (let t = 1; t <= N; t++) {
        let minCostForT = Infinity; let bestTauForT = 0;
        // Pruning candidates based on the PELT condition
        const R_new = R.filter(tau => F[tau] + getCost(tau, t - 1) <= minCostForT);

        for (const tau of R_new) {
            const cost = F[tau] + getCost(tau, t - 1) + lambda;
            if (cost < minCostForT) {
                minCostForT = cost;
                bestTauForT = tau;
            }
        }
        F[t] = minCostForT;
        P[t] = bestTauForT;

        // Update candidate set for next iteration
        R = R_new.filter(tau => F[tau] + getCost(tau, t - 1) <= F[t]);
        R.push(t);
    }
    const changepoints = []; let current_t = N;
    while (current_t > 0) {
        changepoints.push(current_t);
        current_t = P[current_t];
    }
    changepoints.push(0);
    return changepoints.sort((a,b)=>a-b).filter((v,i,a)=>a.indexOf(v)===i);
}