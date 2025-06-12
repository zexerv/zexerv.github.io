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
    let alignedTrajectories = [];
    let tube = { tubeMin: null, tubeMax: null };

    // --- Core Action Handlers ---
    const handleClear = () => {
        rawTrajectories = [];
        alignedTrajectories = [];
        tube = { tubeMin: null, tubeMax: null };
        clearCanvas(ctx);
        clearCanvas(dtwCtx);
        segmentBtn.disabled = true;
        infoDisplay.textContent = 'Draw one or more trajectories and click "Align".';
    };

    const handleAlign = () => {
        if (rawTrajectories.length === 0 || rawTrajectories.flat().length < 2) {
            infoDisplay.textContent = "Please draw at least one trajectory.";
            return;
        }
        infoDisplay.textContent = "Aligning...";
        segmentBtn.disabled = true;

        setTimeout(() => {
            const factor = parseInt(downsampleSlider.value, 10);
            const downsampled = rawTrajectories.map(t => downsample(t, factor));
            const { aligned, costMatrices } = align(downsampled);
            
            alignedTrajectories = aligned;
            if (aligned.length > 0 && aligned.flat().length > 0) {
                 tube = calculateTube(alignedTrajectories);
                 segmentBtn.disabled = false;
                 infoDisplay.textContent = `Alignment complete. Found ${costMatrices.length} DTW comparisons. Ready for segmentation.`;
            } else {
                 infoDisplay.textContent = `Alignment failed. Please clear and try again.`;
            }
            
            drawState(); 
            drawCostMatrices(dtwCtx, costMatrices);
        }, 10);
    };

    const handleSegment = () => {
        if (alignedTrajectories.length === 0) {
            infoDisplay.textContent = "Please align trajectories first.";
            return;
        }
        infoDisplay.textContent = "Segmenting...";
        
        setTimeout(() => {
            const normalizedTube = normalizeTube(tube);
            const lambdaSEGDP = getLogValue(parseFloat(lambdaSegdpSlider.value), 0.01, 100);
            const lambdaPELT = getLogValue(parseFloat(lambdaPeltSlider.value), 0.01, 100);

            const segdp_cps = runSEGDP(normalizedTube.tubeMin, normalizedTube.tubeMax, lambdaSEGDP);
            const fastsegdp_cps = runFastSEGDP(normalizedTube.tubeMin, normalizedTube.tubeMax, lambdaPELT);
            
            drawState({ segdp_cps, fastsegdp_cps });

            infoDisplay.textContent = `SEGDP: ${segdp_cps.length - 1} segments\nFastSEGDP: ${fastsegdp_cps.length - 1} segments`;
        }, 10);
    };
    
    // --- Drawing & Visualization ---
    const drawState = (segmentations = {}) => {
        clearCanvas(ctx);
        drawTrajectories(ctx, rawTrajectories, '#475569', 1.5);
        if (alignedTrajectories.length > 0 && tube.tubeMin) {
            drawTube(ctx, tube.tubeMin, tube.tubeMax);
        }
        if (segmentations.segdp_cps) {
            drawSegmentation(ctx, segmentations.segdp_cps, tube.tubeMin, tube.tubeMax);
            drawVerticalChangepoints(ctx, segmentations.segdp_cps, alignedTrajectories, '#63B3ED', 'dotted');
        }
        if (segmentations.fastsegdp_cps) {
            drawVerticalChangepoints(ctx, segmentations.fastsegdp_cps, alignedTrajectories, '#68D391', 'dashed');
        }
        if (segmentations.segdp_cps || segmentations.fastsegdp_cps) {
            drawLegend(ctx);
        }
    };
    
    // --- Event Listeners Setup ---
    const setupEventListeners = () => {
        const startDrawing = (e) => {
            isDrawing = true;
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
        ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
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

const drawSegmentation = (ctx, cps, tMin, tMax) => {
    if (!cps || cps.length < 2 || !tMin || tMin.length === 0) return;
    ctx.lineWidth = 3;
    // Min boundary (continuous piecewise line)
    ctx.strokeStyle = '#63B3ED';
    ctx.beginPath();
    ctx.moveTo(tMin[cps[0]].x, tMin[cps[0]].y);
    for(let i = 1; i < cps.length; i++) {
        const endIdx = cps[i] === tMin.length ? cps[i] - 1 : cps[i];
        if (tMin[endIdx]) ctx.lineTo(tMin[endIdx].x, tMin[endIdx].y);
    }
    ctx.stroke();
    // Max boundary
    ctx.strokeStyle = '#4299E1';
    ctx.beginPath();
    ctx.moveTo(tMax[cps[0]].x, tMax[cps[0]].y);
    for(let i = 1; i < cps.length; i++) {
        const endIdx = cps[i] === tMax.length ? cps[i] - 1 : cps[i];
        if (tMax[endIdx]) ctx.lineTo(tMax[endIdx].x, tMax[endIdx].y);
    }
    ctx.stroke();
};

const drawVerticalChangepoints = (ctx, cps, trajs, color, style) => {
    if (!trajs || trajs.length === 0 || !cps) return;
    const refTraj = trajs[0];
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

const drawCostMatrices = (ctx, matrices) => {
    clearCanvas(ctx);
    if (!matrices || matrices.length === 0) return;
    const canvasWidth = ctx.canvas.getBoundingClientRect().width;
    const canvasHeight = ctx.canvas.getBoundingClientRect().height;
    const totalMatrices = matrices.length;
    const matrixHeight = canvasHeight / totalMatrices;
    matrices.forEach((matrix, index) => {
        if (!matrix || matrix.length === 0) return;
        const n = matrix.length; const m = matrix[0].length;
        const w = canvasWidth / m; const h = matrixHeight / n;
        const yOffset = index * matrixHeight;
        const maxCost = matrix[n-1][m-1];
        if (maxCost === 0) return;
        for (let i = 0; i < n; i++) for (let j = 0; j < m; j++) {
            const value = matrix[i][j] / maxCost;
            const colorVal = Math.floor((1 - value) * 200) + 55;
            ctx.fillStyle = `rgb(${colorVal}, ${colorVal}, ${colorVal})`;
            const yPos = yOffset + (matrixHeight - (i + 1) * h);
            ctx.fillRect(j * w, yPos, w, h);
        }
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
    if (trajectories.length <= 1) return { aligned: trajectories, costMatrices: [] };
    const refIndex = trajectories.map(t => t.length).indexOf(Math.max(...trajectories.map(t => t.length)));
    const refTraj = trajectories[refIndex];
    const aligned = [refTraj];
    const costMatrices = [];
    for (let i = 0; i < trajectories.length; i++) {
        if (i === refIndex) continue;
        const trajToAlign = trajectories[i];
        if (trajToAlign.length === 0) continue;
        const costMatrix = dtw(refTraj, trajToAlign);
        costMatrices.push(costMatrix);
        const path = backtrackDtw(costMatrix);
        if(path.length === 0) continue;
        const warpedTraj = path.map(([refI, trajI]) => trajToAlign[trajI]);
        aligned.push(warpedTraj);
    }
    return { aligned, costMatrices };
}

const calculateTube = (trajs) => {
    if (trajs.length === 0 || trajs[0].length === 0) return {tubeMin: [], tubeMax: []};
    const n = trajs[0].length;
    const tMin = Array.from({length:n},()=>({x:Infinity,y:Infinity}));
    const tMax = Array.from({length:n},()=>({x:-Infinity,y:-Infinity}));
    for(let i=0;i<n;i++) for(let j=0;j<trajs.length;j++) {
        if (!trajs[j] || !trajs[j][i]) continue;
        const p=trajs[j][i];
        tMin[i].x=Math.min(tMin[i].x,p.x); tMin[i].y=Math.min(tMin[i].y,p.y);
        tMax[i].x=Math.max(tMax[i].x,p.x); tMax[i].y=Math.max(tMax[i].y,p.y);
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
        const min_s=(tMin[end][dim]-tMin[start][dim])/(len-1);
        const max_s=(tMax[end][dim]-tMax[start][dim])/(len-1);
        for(let i=0;i<len;i++){
            cost+=Math.pow(tMin[start+i][dim]-(tMin[start][dim]+min_s*i),2);
            cost+=Math.pow(tMax[start+i][dim]-(tMax[start][dim]+max_s*i),2);
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
    const cps=[N]; let t=N-1,m=bestM;
    while(m>0&&t>=0){let s=bp[t][m]; if(s===-1)break; cps.push(s); t=s-1; m--;}
    return cps.sort((a,b)=>a-b).filter((v,i,a)=>a.indexOf(v)===i);
}

const getSegmentCostPELT = (start, end, tMin, tMax) => {
    let cost = 0;
    for(const dim of ['x', 'y']) {
        const minPts = tMin.slice(start, end + 1).map(p => p[dim]);
        const maxPts = tMax.slice(start, end + 1).map(p => p[dim]);
        const n=minPts.length; if(n===0)continue;
        let sxm=0,sym=0,sxym=0,sxxm=0; let sxa=0,sya=0,sxya=0,sxxa=0;
        for(let i=0;i<n;i++){
            sxm+=i;sym+=minPts[i];sxym+=i*minPts[i];sxxm+=i*i;
            sxa+=i;sya+=maxPts[i];sxya+=i*maxPts[i];sxxa+=i*i;
        }
        const den_m=n*sxxm-sxm*sxm; const den_a=n*sxxa-sxa*sxa;
        const slope_m=den_m===0?0:(n*sxym-sxm*sym)/den_m; const int_m=(sym-slope_m*sxm)/n;
        const slope_a=den_a===0?0:(n*sxya-sxa*sya)/den_a; const int_a=(sya-slope_a*sxa)/n;
        for(let i=0;i<n;i++){
            cost+=Math.pow(minPts[i]-(slope_m*i+int_m),2);
            cost+=Math.pow(maxPts[i]-(slope_a*i+int_a),2);
        }
    }
    return cost;
};

function runFastSEGDP(tubeMin, tubeMax, lambda) {
    const N = tubeMin.length; if (N === 0) return [];
    const costCache = {};
    const getCost = (start, end) => {
        const key = `${start}-${end}`;
        if (key in costCache) return costCache[key];
        return costCache[key] = getSegmentCostPELT(start, end, tubeMin, tubeMax);
    };
    const F = Array(N + 1).fill(Infinity);
    const P = Array(N + 1).fill(0);
    F[0] = -lambda;
    let R = [0];
    for (let t = 1; t <= N; t++) {
        let minCostForT = Infinity; let bestTauForT = 0;
        for (const tau of R) {
            const cost = F[tau] + getCost(tau, t - 1) + lambda;
            if (cost < minCostForT) { minCostForT = cost; bestTauForT = tau; }
        }
        F[t] = minCostForT; P[t] = bestTauForT;
        const R_new = [];
        for (const tau of R) {
            if (F[tau] + getCost(tau, t - 1) <= F[t]) { R_new.push(tau); }
        }
        R_new.push(t);
        R = R_new;
    }
    const changepoints = []; let current_t = N;
    while (current_t > 0) {
        changepoints.push(current_t);
        current_t = P[current_t];
    }
    changepoints.push(0);
    return changepoints.sort((a,b) => a-b).filter((v,i,a) => a.indexOf(v)===i);
}
