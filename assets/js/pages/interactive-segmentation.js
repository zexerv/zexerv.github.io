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

    // --- Utility & Setup ---
    const resizeAllCanvases = () => {
        const dpr = window.devicePixelRatio || 1;
        [canvas, dtwCanvas].forEach(c => {
            const rect = c.getBoundingClientRect();
            c.width = rect.width * dpr;
            c.height = rect.height * dpr;
            c.getContext('2d').scale(dpr, dpr);
        });
        // Redraw content after resize
        drawState();
    };

    const getMousePos = (evt) => {
        const rect = canvas.getBoundingClientRect();
        return { x: evt.clientX - rect.left, y: evt.clientY - rect.top };
    };

    // --- Core Action Handlers ---
    const handleClear = () => {
        rawTrajectories = [];
        alignedTrajectories = [];
        tube = { tubeMin: null, tubeMax: null };
        clearCanvas(ctx);
        clearCanvas(dtwCtx);
        segmentBtn.disabled = true;
        infoDisplay.textContent = 'Draw a trajectory and click "Align Trajectories".';
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
            const startTime = performance.now();

            const downsampled = rawTrajectories.map(t => downsample(t, factor));
            const { aligned, costMatrix } = align(downsampled);
            const alignmentTime = performance.now() - startTime;
            
            alignedTrajectories = aligned;
            if (aligned.length > 0 && aligned[0].length > 0) {
                 tube = calculateTube(alignedTrajectories);
                 segmentBtn.disabled = false;
                 infoDisplay.textContent = `Alignment Complete: ${alignmentTime.toFixed(1)} ms.\nReady for segmentation.`;
            } else {
                 infoDisplay.textContent = `Alignment failed. Please try again.`;
            }
            
            drawState();
            drawCostMatrix(dtwCtx, costMatrix);
        }, 50);
    };

    const handleSegment = () => {
        if (alignedTrajectories.length === 0) {
            infoDisplay.textContent = "Please align trajectories first.";
            return;
        }
        infoDisplay.textContent = "Segmenting...";
        
        setTimeout(() => {
            const lambdaSEGDP = parseFloat(lambdaSegdpSlider.value);
            const lambdaPELT = parseFloat(lambdaPeltSlider.value);

            const startTimeSEGDP = performance.now();
            const segdp_cps = runSEGDP(tube.tubeMin, tube.tubeMax, lambdaSEGDP);
            const segdpTime = performance.now() - startTimeSEGDP;

            const startTimePELT = performance.now();
            const fastsegdp_cps = runFastSEGDP(tube.tubeMin, tube.tubeMax, lambdaPELT);
            const peltTime = performance.now() - startTimePELT;
            
            drawState({ segdp_cps, fastsegdp_cps });

            infoDisplay.textContent = `SEGDP: ${segdp_cps.length - 1} segments (${segdpTime.toFixed(1)} ms)\nFastSEGDP: ${fastsegdp_cps.length - 1} segments (${peltTime.toFixed(1)} ms)`;
        }, 50);
    };
    
    // --- Drawing & Visualization ---
    const drawState = (segmentations = {}) => {
        clearCanvas(ctx);
        if (alignedTrajectories.length > 0) {
            drawTrajectories(ctx, alignedTrajectories, '#FDBA74', 1); // Aligned are orange
            if(tube.tubeMin) drawTube(ctx, tube.tubeMin, tube.tubeMax);
        } else {
            drawTrajectories(ctx, rawTrajectories, '#A0AEC0', 1.5); // Raw are grey
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
        window.addEventListener('resize', resizeAllCanvases);

        // Drawing listeners
        const start = (e) => { isDrawing = true; rawTrajectories.push([getMousePos(e)]); };
        const move = (e) => { if(isDrawing) { rawTrajectories[rawTrajectories.length - 1].push(getMousePos(e)); drawState(); } };
        const end = () => { if(isDrawing) { isDrawing = false; alignedTrajectories = []; segmentBtn.disabled = true; } };
        
        canvas.addEventListener('mousedown', start);
        canvas.addEventListener('mousemove', move);
        canvas.addEventListener('mouseup', end);
        canvas.addEventListener('mouseout', end);
        
        const touchStart = (e) => { e.preventDefault(); start(e.touches[0]); };
        const touchMove = (e) => { e.preventDefault(); move(e.touches[0]); };
        canvas.addEventListener('touchstart', touchStart);
        canvas.addEventListener('touchmove', touchMove);
        canvas.addEventListener('touchend', end);
        
        // Button Listeners
        alignBtn.addEventListener('click', handleAlign);
        segmentBtn.addEventListener('click', handleSegment);
        clearBtn.addEventListener('click', handleClear);
        
        // Slider Listeners
        downsampleSlider.addEventListener('input', () => downsampleValueSpan.textContent = downsampleSlider.value);
        lambdaSegdpSlider.addEventListener('input', () => lambdaSegdpValueSpan.textContent = lambdaSegdpSlider.value);
        lambdaPeltSlider.addEventListener('input', () => lambdaPeltValueSpan.textContent = `${(parseFloat(lambdaPeltSlider.value)/1000000).toFixed(1)}M`);
    };

    // --- Initial Call ---
    handleClear(); // Set initial state
    resizeAllCanvases();
    setupEventListeners();
    // Set initial slider text
    downsampleValueSpan.textContent = downsampleSlider.value;
    lambdaSegdpValueSpan.textContent = lambdaSegdpSlider.value;
    lambdaPeltValueSpan.textContent = `${(parseFloat(lambdaPeltSlider.value)/1000000).toFixed(1)}M`;
});


// --- ALGORITHMS & VISUALIZATION HELPERS (can be outside DOMContentLoaded) ---

const clearCanvas = (ctx) => ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);

const drawTrajectories = (ctx, trajs, color, lineWidth) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    trajs.forEach(traj => {
        if (traj.length < 2) return;
        ctx.beginPath();
        ctx.moveTo(traj[0].x, traj[0].y);
        for (let i = 1; i < traj.length; i++) ctx.lineTo(traj[i].x, traj[i].y);
        ctx.stroke();
    });
};

const drawTube = (ctx, tMin, tMax) => {
    ctx.fillStyle = 'rgba(100, 116, 139, 0.3)';
    ctx.beginPath();
    ctx.moveTo(tMax[0].x, tMax[0].y);
    for (let i = 1; i < tMax.length; i++) ctx.lineTo(tMax[i].x, tMax[i].y);
    for (let i = tMin.length - 1; i >= 0; i--) ctx.lineTo(tMin[i].x, tMin[i].y);
    ctx.closePath();
    ctx.fill();
};

const drawCostMatrix = (ctx, matrix) => {
    if (!matrix || matrix.length === 0) return;
    const n = matrix.length;
    const m = matrix[0].length;
    const w = ctx.canvas.getBoundingClientRect().width / m;
    const h = ctx.canvas.getBoundingClientRect().height / n;
    const maxCost = matrix[n-1][m-1];
    if(maxCost === 0) return;
    for (let i = 0; i < n; i++) {
        for (let j = 0; j < m; j++) {
            const value = matrix[i][j] / maxCost;
            const colorVal = Math.floor((1 - value) * 200) + 55; // Darker theme heatmap
            ctx.fillStyle = `rgb(${colorVal}, ${colorVal}, ${colorVal})`;
            ctx.fillRect(j * w, i * h, w, h);
        }
    }
};

const drawSegmentation = (ctx, changepoints, tMin, tMax) => {
    if (!changepoints || changepoints.length < 2) return;
    ctx.lineWidth = 3;
    // Min boundary (continuous piecewise line)
    ctx.strokeStyle = '#63B3ED';
    ctx.beginPath();
    ctx.moveTo(tMin[changepoints[0]].x, tMin[changepoints[0]].y);
    for(let i = 1; i < changepoints.length; i++) {
        const endIdx = changepoints[i] === tMin.length ? changepoints[i] - 1 : changepoints[i];
        if (tMin[endIdx]) ctx.lineTo(tMin[endIdx].x, tMin[endIdx].y);
    }
    ctx.stroke();
    // Max boundary
    ctx.strokeStyle = '#4299E1';
    ctx.beginPath();
    ctx.moveTo(tMax[changepoints[0]].x, tMax[changepoints[0]].y);
    for(let i = 1; i < changepoints.length; i++) {
        const endIdx = changepoints[i] === tMax.length ? changepoints[i] - 1 : changepoints[i];
        if (tMax[endIdx]) ctx.lineTo(tMax[endIdx].x, tMax[endIdx].y);
    }
    ctx.stroke();
};

const drawVerticalChangepoints = (ctx, changepoints, trajs, color, style) => {
    if (!trajs || trajs.length === 0 || !changepoints) return;
    const refTraj = trajs[0];
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    if (style === 'dashed') ctx.setLineDash([5, 5]);
    else if (style === 'dotted') ctx.setLineDash([2, 3]);
    
    changepoints.forEach(cp_idx => {
        if (cp_idx > 0 && cp_idx < refTraj.length) {
            const point = refTraj[cp_idx];
            ctx.beginPath();
            ctx.moveTo(point.x, 0);
            ctx.lineTo(point.x, ctx.canvas.getBoundingClientRect().height);
            ctx.stroke();
        }
    });
    ctx.setLineDash([]); // Reset line dash
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

const downsample = (trajectory, factor) => {
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
};

const euclideanDistance = (p1, p2) => Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));

function dtw(refSeq, trajSeq) {
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
}

function backtrackDtw(costMatrix) {
    if (!costMatrix || costMatrix.length === 0) return [];
    let path = [];
    let i = costMatrix.length - 1; let j = costMatrix[0].length - 1;
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
        const costMatrix = dtw(refTraj, trajectories[i]);
        if (i === (refIndex > 0 ? 0 : 1)) primaryCostMatrix = costMatrix;
        const path = backtrackDtw(costMatrix);
        const warpedTraj = [];
        let lastTrajIndex = 0;
        const refToTrajMap = {};
        path.forEach(([refI, trajI]) => { if (!(refI in refToTrajMap)) refToTrajMap[refI] = trajI; });
        for (let j = 0; j < refTraj.length; j++) {
             if (j in refToTrajMap) lastTrajIndex = refToTrajMap[j];
             warpedTraj.push(trajectories[i][lastTrajIndex]);
        }
        aligned.push(warpedTraj);
    }
    return { aligned, costMatrix: primaryCostMatrix };
}

function calculateTube(trajs) {
    const n = trajs[0].length;
    const tMin = Array.from({length:n},()=>({x:Infinity,y:Infinity}));
    const tMax = Array.from({length:n},()=>({x:-Infinity,y:-Infinity}));
    for(let i=0;i<n;i++) for(let j=0;j<trajs.length;j++) {
        const p=trajs[j][i]; if(!p) continue;
        tMin[i].x=Math.min(tMin[i].x,p.x); tMin[i].y=Math.min(tMin[i].y,p.y);
        tMax[i].x=Math.max(tMax[i].x,p.x); tMax[i].y=Math.max(tMax[i].y,p.y);
    }
    return {tubeMin:tMin,tubeMax:tMax};
}

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

const linearFit=(pts)=>{
    const n=pts.length; if(n===0)return{error:0}; let sx=0,sy=0,sxy=0,sxx=0;
    for(let i=0;i<n;i++){sx+=i;sy+=pts[i];sxy+=i*pts[i];sxx+=i*i;}
    const slope=(n*sxy-sx*sy)/(n*sxx-sx*sx); const intercept=(sy-slope*sx)/n; let err=0;
    for(let i=0;i<n;i++)err+=Math.pow(pts[i]-(slope*i+intercept),2); return {error:err};
};
const getSegmentCostPELT=(s,e,tMin,tMax)=>linearFit(tMin.slice(s,e+1).map(p=>p.x)).error+linearFit(tMin.slice(s,e+1).map(p=>p.y)).error+linearFit(tMax.slice(s,e+1).map(p=>p.x)).error+linearFit(tMax.slice(s,e+1).map(p=>p.y)).error;

function runFastSEGDP(tubeMin, tubeMax, lambda) {
    const N=tubeMin.length; if(N===0) return [];
    const F=Array(N+1).fill(Infinity); const P=Array(N+1).fill(0);
    F[0]=-lambda; let R=[0]; const costCache={}; const getCost=(s,e)=>{const k=`${s}-${e}`; if(k in costCache)return costCache[k]; return costCache[k]=getSegmentCostPELT(s,e,tubeMin,tubeMax);};
    for(let t=1;t<=N;t++){
        let best_tau=0,min_cost=Infinity;
        for(const tau of R){const cost=F[tau]+getCost(tau,t-1)+lambda; if(cost<min_cost){min_cost=cost;best_tau=tau;}}
        F[t]=min_cost;P[t]=best_tau;
        R=R.filter(tau=>F[tau]+getCost(tau,t-1)<=F[t]); R.push(t);
    }
    const cps=[N]; let t=N;
    while(t>0){let cp=P[t]; cps.push(cp); t=cp;}
    return cps.sort((a,b)=>a-b).filter((v,i,a)=>a.indexOf(v)===i);
}