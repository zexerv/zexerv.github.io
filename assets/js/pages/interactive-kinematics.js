import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// --- Configuration & Constants ---
// ... (Keep all constants and DH parameters the same as before) ...
const DH_PARAMS_UR5E = [
    { a: 0.0,    alpha: Math.PI / 2, d: 0.1625, theta_offset: 0.0 },
    { a: -0.425, alpha: 0.0,         d: 0.0,    theta_offset: 0.0 },
    { a: -0.3922,alpha: 0.0,         d: 0.0,    theta_offset: 0.0 },
    { a: 0.0,    alpha: Math.PI / 2, d: 0.1333, theta_offset: 0.0 },
    { a: 0.0,    alpha: -Math.PI/ 2, d: 0.0997, theta_offset: 0.0 },
    { a: 0.0,    alpha: 0.0,         d: 0.0996, theta_offset: 0.0 }
];
const TCP_Z_OFFSET = 0.1565;
const H_FLANGE_TCP = new THREE.Matrix4().makeTranslation(0, 0, TCP_Z_OFFSET);
const INV_H_FLANGE_TCP = new THREE.Matrix4().makeTranslation(0, 0, -TCP_Z_OFFSET);
const d1 = DH_PARAMS_UR5E[0].d;
const a2 = DH_PARAMS_UR5E[1].a;
const a3 = DH_PARAMS_UR5E[2].a;
const d4 = DH_PARAMS_UR5E[3].d;
const d5 = DH_PARAMS_UR5E[4].d;
const d6 = DH_PARAMS_UR5E[5].d;
const len_a2 = Math.abs(a2);
const len_a3 = Math.abs(a3);
const tol_zero = 1e-9;
const tol_singularity = 1e-7;
const tol_compare = 1e-6;
const tol_geom = 1e-6;
const PRIMARY_LINK_COLOR = 0xeeeeee;
const PRIMARY_JOINT_COLOR = 0xffffff;
const IK_LINK_COLOR = 0xaaaaaa;
const IK_JOINT_COLOR = 0x888888;
const TARGET_TCP_COLOR = 0xff8800;
const AXIS_LENGTH = 0.05;
const JOINT_RADIUS = 0.03;
const LINK_RADIUS = 0.015; // Thinner links
const IK_SOLUTION_OPACITY = 0.4;
const initialJointAngles = [0.0, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, -Math.PI / 2, 0.0];


// --- Global Variables ---
// (Declare them here, assign inside init or DOMContentLoaded)
let scene, camera, renderer, controls;
let robotInstancesGroup, targetTCPGroup;
let jointSliders = [], valueDisplays = [];
let canvasContainer, canvas, slidersContainer, tcpPoseOutput, ikSolutionCount, ikSolutionsOutput, loadingMessage, errorMessage; // Declare DOM element vars

// --- Helper Functions ---
// ... dhMatrix ...
// ... forwardKinematics ...
// ... inverseKinematics ...
// ... areAnglesClose ...
// ... createRobotVisualizationInstance ...
// ... formatMatrix ...
// ... generateDistinctColors ...
// (Keep ALL the helper functions from the previous version exactly the same)
// DH Matrix function (Unchanged)
function dhMatrix(a, alpha, d, theta) {
    const cos_t = Math.cos(theta);
    const sin_t = Math.sin(theta);
    const cos_a = Math.cos(alpha);
    const sin_a = Math.sin(alpha);
    const matrix = new THREE.Matrix4();
    matrix.set(
        cos_t, -sin_t * cos_a, sin_t * sin_a, a * cos_t,
        sin_t, cos_t * cos_a, -cos_t * sin_a, a * sin_t,
        0, sin_a, cos_a, d,
        0, 0, 0, 1
    );
    return matrix;
}

// Forward Kinematics function (Unchanged, returns transforms now)
function forwardKinematics(jointAngles) {
    if (jointAngles.length !== DH_PARAMS_UR5E.length) {
        console.error("Angle count mismatch");
        return { points: [], T0_TCP: null, T0_Flange: null, transforms: [] };
    }
    let transforms = [new THREE.Matrix4()];
    let T_prev = transforms[0].clone();
    for (let i = 0; i < DH_PARAMS_UR5E.length; i++) {
        const p = DH_PARAMS_UR5E[i];
        const theta = jointAngles[i] + p.theta_offset;
        const T_i_minus_1_to_i = dhMatrix(p.a, p.alpha, p.d, theta);
        const T_curr = new THREE.Matrix4().multiplyMatrices(T_prev, T_i_minus_1_to_i);
        transforms.push(T_curr.clone());
        T_prev = T_curr;
    }
    const T0_Flange = transforms[transforms.length - 1];
    const T0_TCP = new THREE.Matrix4().multiplyMatrices(T0_Flange, H_FLANGE_TCP);
    // Extract points
    const points = transforms.map(T => {
        const position = new THREE.Vector3();
        T.decompose(position, new THREE.Quaternion(), new THREE.Vector3());
        return position;
    });
    // Add TCP point
    const tcpPos = new THREE.Vector3();
    T0_TCP.decompose(tcpPos, new THREE.Quaternion(), new THREE.Vector3());
    points.push(tcpPos);

    return { points, T0_TCP, T0_Flange, transforms };
}

// *** Inverse Kinematics function (Ported from Python) ***
function inverseKinematics(T_desired_TCP) {
    const solutions = [];
    const T_flange = new THREE.Matrix4().multiplyMatrices(T_desired_TCP, INV_H_FLANGE_TCP);

    const P60 = new THREE.Vector3().setFromMatrixPosition(T_flange);
    const R06 = new THREE.Matrix4().extractRotation(T_flange);
    const r = R06.elements; // Matrix elements in column-major order

    // Target Flange Position
    const pxd = P60.x;
    const pyd = P60.y;
    const pzd = P60.z;

    // Target Flange Orientation elements (using THREE.js column-major format)
    const r11d = r[0], r21d = r[1], r31d = r[2];
    const r12d = r[4], r22d = r[5], r32d = r[6];
    const r13d = r[8], r23d = r[9], r33d = r[10];

    // Calculate Wrist Center Point (P50)
    const P50 = P60.clone().sub(new THREE.Vector3(r13d, r23d, r33d).multiplyScalar(d6));
    const P50x = P50.x, P50y = P50.y, P50z = P50.z;

    // --- Theta 1 ---
    const A = P50y;
    const B = P50x;
    const dist_sq_xy = B * B + A * A;

    if (dist_sq_xy < d4 * d4 - tol_geom) { /* console.log("IK T1: Below d4 range"); */ return []; }
    if (Math.abs(B) < tol_singularity && Math.abs(A) < tol_singularity) { /* console.log("IK T1: Singularity at base"); */ return []; }

    const sqrt_arg_t1 = dist_sq_xy - d4 * d4;
    const sqrt_val_t1 = Math.sqrt(Math.max(0, sqrt_arg_t1));

    const term1_t1 = Math.atan2(A, B);
    const term2_t1 = Math.atan2(sqrt_val_t1, d4);

    const theta1_sol = [
        term1_t1 - term2_t1,
        term1_t1 + term2_t1 + Math.PI // Adjusted from python version based on common UR IK forms
    ];


    for (let t1_idx = 0; t1_idx < theta1_sol.length; t1_idx++) {
        const t1 = theta1_sol[t1_idx];
        const s1 = Math.sin(t1), c1 = Math.cos(t1);

        // --- Theta 5 ---
        const M_val = (r13d * s1 - r23d * c1);
        const M = Math.max(-1.0, Math.min(1.0, M_val));

        let t5_options = [];
        if (Math.abs(Math.abs(M) - 1.0) < tol_singularity) {
            t5_options.push(M > 0 ? 0 : Math.PI);
        } else {
            const sqrt_M_term = Math.sqrt(1 - M * M);
            t5_options = [
                Math.atan2(sqrt_M_term, M),
                Math.atan2(-sqrt_M_term, M)
            ];
        }

        for (let t5_idx = 0; t5_idx < t5_options.length; t5_idx++) {
            const t5 = t5_options[t5_idx];
            const s5 = Math.sin(t5), c5 = Math.cos(t5);

            // --- Theta 6 ---
            let t6 = 0.0;
            if (Math.abs(s5) < tol_singularity) {
                t6 = 0.0;
            } else {
                 const D_t6 = (r12d * s1 - r22d * c1);
                 const E_t6 = (r11d * s1 - r21d * c1);
                 // Avoid atan2(0,0) for t6 as well
                 if (Math.abs(D_t6) < tol_singularity && Math.abs(E_t6) < tol_singularity && Math.abs(s5) > tol_singularity) {
                      // This case implies alignment where multiple t6 could work, often arises with t5=pi/2
                      //console.warn("IK T6: Singularity atan2(0,0) case, setting t6=0");
                      t6 = 0.0; // Set specific value, though technically depends on previous joints
                 } else {
                    t6 = Math.atan2(-D_t6 / s5, E_t6 / s5);
                 }
            }
            const s6 = Math.sin(t6), c6 = Math.cos(t6);

            // --- Theta 3 ---
            const T65 = dhMatrix(DH_PARAMS_UR5E[5].a, DH_PARAMS_UR5E[5].alpha, DH_PARAMS_UR5E[5].d, t6);
            const T54 = dhMatrix(DH_PARAMS_UR5E[4].a, DH_PARAMS_UR5E[4].alpha, DH_PARAMS_UR5E[4].d, t5);
            const T10 = dhMatrix(DH_PARAMS_UR5E[0].a, DH_PARAMS_UR5E[0].alpha, DH_PARAMS_UR5E[0].d, t1);

            const T46_inv = new THREE.Matrix4().multiplyMatrices(T54, T65).invert();
            const T04 = new THREE.Matrix4().multiplyMatrices(T_flange, T46_inv);
            const T14 = new THREE.Matrix4().multiplyMatrices(new THREE.Matrix4().copy(T10).invert(), T04);

            const P41 = new THREE.Vector3().setFromMatrixPosition(T14);
            const P41x = P41.x;
            const P41y = P41.y;

            const dist_sq_14 = P41x * P41x + P41y * P41y;
            const cos_t3_arg_num = dist_sq_14 - a2 * a2 - a3 * a3;
            const cos_t3_arg_den = 2 * a2 * a3;

            if (Math.abs(cos_t3_arg_den) < tol_zero) { continue; }
            const cos_t3_arg = cos_t3_arg_num / cos_t3_arg_den;
            if (Math.abs(cos_t3_arg) > 1.0 + tol_geom) { continue; }
            const cos_t3 = Math.max(-1.0, Math.min(1.0, cos_t3_arg));

            const sqrt_arg_t3 = 1.0 - cos_t3 * cos_t3;
            const sqrt_val_t3 = Math.sqrt(Math.max(0, sqrt_arg_t3));

            const t3_sol = [
                Math.atan2(sqrt_val_t3, cos_t3),
                Math.atan2(-sqrt_val_t3, cos_t3)
            ];


            for (let t3_idx = 0; t3_idx < t3_sol.length; t3_idx++) {
                const t3 = t3_sol[t3_idx];
                const s3 = Math.sin(t3), c3 = cos_t3; // Use the clipped cos_t3

                // --- Theta 2 ---
                const term2_num_t2 = a3 * s3;
                const term2_den_t2 = a2 + a3 * c3; // Denominator for atan2 term

                const term1_t2 = Math.atan2(-P41y, -P41x);
                const term2_t2 = Math.atan2(term2_num_t2, term2_den_t2);

                const t2 = term1_t2 - term2_t2;

                // --- Theta 4 ---
                const T21 = dhMatrix(DH_PARAMS_UR5E[1].a, DH_PARAMS_UR5E[1].alpha, DH_PARAMS_UR5E[1].d, t2);
                const T32 = dhMatrix(DH_PARAMS_UR5E[2].a, DH_PARAMS_UR5E[2].alpha, DH_PARAMS_UR5E[2].d, t3);
                const T03 = new THREE.Matrix4().multiplyMatrices(T10, T21).multiply(T32);
                const T34 = new THREE.Matrix4().multiplyMatrices(new THREE.Matrix4().copy(T03).invert(), T04);

                const R34 = new THREE.Matrix4().extractRotation(T34);
                const r34 = R34.elements;
                const t4 = Math.atan2(r34[1], r34[0]);

                // --- Assemble Solution ---
                const sol_angles_raw = [t1, t2, t3, t4, t5, t6];
                const sol_angles_normalized = sol_angles_raw.map(a => {
                    let wrapped = (a + Math.PI) % (2 * Math.PI);
                    if (wrapped < 0) wrapped += 2 * Math.PI;
                    return wrapped - Math.PI;
                });

                 // --- Verification (Basic FK check) ---
                 try {
                     const { T0_TCP: T_check } = forwardKinematics(sol_angles_normalized);
                     if (T_check) {
                        // Check position distance
                        const pos_check = new THREE.Vector3().setFromMatrixPosition(T_check);
                        const pos_target = new THREE.Vector3().setFromMatrixPosition(T_desired_TCP);
                        const posDiff = pos_check.distanceTo(pos_target);

                        // Check orientation (angle between Z axes - simpler than full quaternion diff)
                        const z_check = new THREE.Vector3(T_check.elements[8], T_check.elements[9], T_check.elements[10]).normalize();
                        const z_target = new THREE.Vector3(T_desired_TCP.elements[8], T_desired_TCP.elements[9], T_desired_TCP.elements[10]).normalize();
                        const angleDiff = z_check.angleTo(z_target); // Radians

                        if (posDiff < tol_compare * 100 && angleDiff < tol_compare * 100) { // Looser tolerance for web demo
                             solutions.push(sol_angles_normalized);
                        } else {
                             // console.log(`IK Solution failed verification: P ${posDiff.toFixed(5)}, A ${angleDiff.toFixed(5)}`, sol_angles_normalized.map(a => a.toFixed(2)));
                        }
                     }
                 } catch (e) { /* FK failed */ }

            } // t3 loop
        } // t5 loop
    } // t1 loop

    // Filter duplicates
    const unique_solutions = [];
    for (const sol of solutions) {
        let is_unique = true;
        for (const unique_sol of unique_solutions) {
            if (areAnglesClose(sol, unique_sol, tol_compare)) {
                is_unique = false;
                break;
            }
        }
        if (is_unique) {
            unique_solutions.push(sol);
        }
    }

    return unique_solutions;
}

// Helper to compare joint angle arrays
function areAnglesClose(angles1, angles2, tolerance = 1e-4) {
    if (!angles1 || !angles2 || angles1.length !== angles2.length) return false;
    for (let i = 0; i < angles1.length; i++) {
        let diff = angles1[i] - angles2[i];
        diff = (diff + Math.PI) % (2 * Math.PI); // Wrap difference to [0, 2pi)
         if (diff < 0) diff += 2 * Math.PI;
         diff = Math.min(diff, 2 * Math.PI - diff); // Use smaller angle difference
        if (diff > tolerance) {
            return false;
        }
    }
    return true;
}


// Function to create ONE instance of the robot visualization
function createRobotVisualizationInstance(jointAngles, isPrimary) {
    const group = new THREE.Group();
    const { points, T0_TCP } = forwardKinematics(jointAngles);

    if (!points || points.length < 8) return null;

    const opacity = isPrimary ? 1.0 : IK_SOLUTION_OPACITY;
    const transparent = !isPrimary;
    const linkColor = isPrimary ? PRIMARY_LINK_COLOR : IK_LINK_COLOR;
    const jointColor = isPrimary ? PRIMARY_JOINT_COLOR : IK_JOINT_COLOR;

    const linkMaterial = new THREE.MeshStandardMaterial({
        color: linkColor, roughness: 0.6, metalness: 0.1,
        opacity: opacity, transparent: transparent, depthWrite: isPrimary
    });
    const jointMaterial = new THREE.MeshStandardMaterial({
         color: jointColor, roughness: 0.5, metalness: 0.2,
         opacity: opacity, transparent: transparent, depthWrite: isPrimary
    });
     const jointGeometry = new THREE.SphereGeometry(JOINT_RADIUS, 12, 8);
     const linkGeometry = new THREE.CylinderGeometry(LINK_RADIUS, LINK_RADIUS, 1, 6);

     for (let i = 0; i < 6; i++) {
         const jointMesh = new THREE.Mesh(jointGeometry, jointMaterial.clone()); // Clone material for potential color changes
         jointMesh.material.color.setHex(JOINT_COLORS[i]); // Use standard joint colors maybe?
         if(!isPrimary) jointMesh.material.color = new THREE.Color(IK_JOINT_COLOR); // Or override for IK
         jointMesh.position.copy(points[i + 1]);
         group.add(jointMesh);
     }

     for (let i = 0; i < 6; i++) {
         const startPoint = points[i];
         const endPoint = points[i + 1];
         const direction = new THREE.Vector3().subVectors(endPoint, startPoint);
         const length = direction.length();
         const midPoint = new THREE.Vector3().addVectors(startPoint, endPoint).multiplyScalar(0.5);

         if (length > 0.001) {
             const linkMesh = new THREE.Mesh(linkGeometry, linkMaterial);
             linkMesh.scale.set(1, length, 1);
             linkMesh.position.copy(midPoint);
             const quaternion = new THREE.Quaternion();
             quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
             linkMesh.setRotationFromQuaternion(quaternion);
             group.add(linkMesh);
         }
     }

     // Add TCP frame only for the primary configuration for clarity
     if (isPrimary && T0_TCP) {
         const tcpFrame = new THREE.AxesHelper(AXIS_LENGTH * 1.5);
         const pos = new THREE.Vector3(); const quat = new THREE.Quaternion();
         T0_TCP.decompose(pos, quat, new THREE.Vector3());
         tcpFrame.position.copy(pos);
         tcpFrame.setRotationFromQuaternion(quat);
         group.add(tcpFrame);
     }

    return group;
}


// --- Main Update Function ---
function updateVisualization() {
    // Ensure DOM elements are available before proceeding
    if (!tcpPoseOutput || !ikSolutionCount || !ikSolutionsOutput || !robotInstancesGroup || !targetTCPGroup) {
        console.error("DOM elements not ready for updateVisualization");
        return;
    }

    const sliderAngles = jointSliders.map(slider => parseFloat(slider.value));
    sliderAngles.forEach((val, i) => {
        if (valueDisplays[i]) valueDisplays[i].textContent = val.toFixed(2);
    });

    // 1. Calculate Target FK Pose
    const { T0_TCP: T_target_TCP } = forwardKinematics(sliderAngles);

    if (!T_target_TCP) {
        tcpPoseOutput.textContent = "Error in FK calculation.";
        ikSolutionCount.textContent = "N/A";
        ikSolutionsOutput.textContent = "N/A";
        robotInstancesGroup.clear();
        targetTCPGroup.clear();
        return;
    }

    // Display Target TCP Pose & Frame
    tcpPoseOutput.textContent = formatMatrix(T_target_TCP);
    targetTCPGroup.clear();
    const targetFrame = new THREE.AxesHelper(AXIS_LENGTH * 2.5);
    const pos = new THREE.Vector3(); const quat = new THREE.Quaternion();
    T_target_TCP.decompose(pos, quat, new THREE.Vector3());
    targetFrame.position.copy(pos);
    targetFrame.setRotationFromQuaternion(quat);
    targetTCPGroup.add(targetFrame);
    const targetMarkerGeo = new THREE.SphereGeometry(JOINT_RADIUS * 0.5, 8, 8);
    const targetMarkerMat = new THREE.MeshBasicMaterial({ color: TARGET_TCP_COLOR });
    const targetMarker = new THREE.Mesh(targetMarkerGeo, targetMarkerMat);
    targetMarker.position.copy(pos);
    targetTCPGroup.add(targetMarker);


    // 2. Calculate ALL IK Solutions
    const ikSolutions = inverseKinematics(T_target_TCP);
    ikSolutionCount.textContent = ikSolutions.length;
    if (ikSolutions.length > 0) {
        ikSolutionsOutput.textContent = ikSolutions.map((sol, idx) =>
            `Sol ${idx + 1}: [${sol.map(a => a.toFixed(2)).join(', ')}]`
        ).join('\n');
    } else {
        ikSolutionsOutput.textContent = "No solutions found (or near singularity).";
    }

    // 3. Visualize all Solutions
    robotInstancesGroup.clear();
    let primaryFoundAmongSolutions = false;

    ikSolutions.forEach((ikAngles) => {
        const isPrimary = areAnglesClose(sliderAngles, ikAngles);
        if (isPrimary) primaryFoundAmongSolutions = true;
        const robotInstance = createRobotVisualizationInstance(ikAngles, isPrimary);
        if (robotInstance) robotInstancesGroup.add(robotInstance);
    });

    if (!primaryFoundAmongSolutions && ikSolutions.length < 8) { // Draw slider config if it wasn't in IK results (max 8 solutions)
         // console.warn("Slider configuration not found among IK solutions, drawing separately.");
         const primaryInstance = createRobotVisualizationInstance(sliderAngles, true);
         if (primaryInstance) robotInstancesGroup.add(primaryInstance);
    } else if (!primaryFoundAmongSolutions && ikSolutions.length >= 8) {
        // console.warn("Slider config not among IK solutions, but max solutions already shown.");
        // Optionally add text indicating the slider config is not shown or is invalid/unreachable by IK
    }
}


// --- Initialization ---
function init() {
    // Assign global DOM element variables here
    canvasContainer = document.getElementById('kinematics-canvas-container');
    canvas = document.getElementById('kinematics-canvas');
    slidersContainer = document.getElementById('joint-sliders');
    tcpPoseOutput = document.getElementById('tcp-pose-output');
    ikSolutionCount = document.getElementById('ik-solution-count');
    ikSolutionsOutput = document.getElementById('ik-solutions-output');
    loadingMessage = document.getElementById('loading-message');
    errorMessage = document.getElementById('error-message');

    // ** Add checks here **
    if (!canvasContainer || !canvas || !slidersContainer || !tcpPoseOutput || !ikSolutionCount || !ikSolutionsOutput || !loadingMessage || !errorMessage) {
         console.error("DOM Initialization Error: One or more required elements not found.");
         if(loadingMessage) loadingMessage.style.display = 'none';
         if(errorMessage) {
            errorMessage.textContent = "Initialization Error: HTML structure mismatch.";
            errorMessage.style.display = 'block';
         }
        return; // Stop initialization if elements are missing
    }

    try {
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x282c34);

        const aspect = canvasContainer.clientWidth / canvasContainer.clientHeight;
        camera = new THREE.PerspectiveCamera(60, aspect, 0.1, 100);
        camera.position.set(0.8, 1.0, 1.2);
        camera.lookAt(0, 0, 0.3);

        renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
        renderer.setSize(canvasContainer.clientWidth, canvasContainer.clientHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        THREE.ColorManagement.enabled = true;

        const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
        scene.add(ambientLight);
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.9);
        directionalLight.position.set(5, 10, 7.5);
        scene.add(directionalLight);

        controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.1;
        controls.target.set(0, 0, 0.3);

        robotInstancesGroup = new THREE.Group();
        scene.add(robotInstancesGroup);
        targetTCPGroup = new THREE.Group();
        scene.add(targetTCPGroup);

        const axesHelper = new THREE.AxesHelper(0.3);
        scene.add(axesHelper);

        createSliders();
        updateVisualization(); // Initial calculation and draw

        loadingMessage.style.display = 'none';
        animate();

    } catch (error) {
        console.error("Initialization failed:", error);
        loadingMessage.style.display = 'none';
        errorMessage.textContent = `Error initializing 3D view: ${error.message}`;
        errorMessage.style.display = 'block';
    }
}

function createSliders() {
     // Ensure slidersContainer exists before manipulating
     if (!slidersContainer) {
          console.error("Cannot create sliders: slidersContainer not found.");
          return;
     }
    slidersContainer.innerHTML = ''; // Clear previous sliders
    jointSliders = [];
    valueDisplays = [];

    for (let i = 0; i < 6; i++) {
        const group = document.createElement('div');
        group.className = 'slider-group';
        const label = document.createElement('label');
        label.htmlFor = `joint${i + 1}-slider`;
        label.textContent = `Joint ${i + 1}:`;
        const slider = document.createElement('input');
        slider.type = 'range';
        slider.id = `joint${i + 1}-slider`;
        slider.min = -Math.PI.toFixed(3);
        slider.max = Math.PI.toFixed(3);
        slider.step = (Math.PI / 180).toFixed(4);
        slider.value = initialJointAngles[i].toFixed(3);
        const valueDisplay = document.createElement('span');
        valueDisplay.className = 'value-display';
        valueDisplay.textContent = parseFloat(slider.value).toFixed(2);
        group.appendChild(label);
        group.appendChild(slider);
        group.appendChild(valueDisplay);
        slidersContainer.appendChild(group);
        jointSliders.push(slider);
        valueDisplays.push(valueDisplay);
        slider.addEventListener('input', updateVisualization);
    }
}


// --- Animation Loop ---
function animate() {
     // Ensure renderer and scene exist before animating
     if (!renderer || !scene || !camera) return;
     requestAnimationFrame(animate);
     if (controls) controls.update(); // Check if controls exist
     renderer.render(scene, camera);
}

// --- Resize Handling ---
function onWindowResize() {
     // Ensure necessary components exist
     if (!canvasContainer || !camera || !renderer) return;
     const width = canvasContainer.clientWidth;
     const height = canvasContainer.clientHeight;
     camera.aspect = width / height;
     camera.updateProjectionMatrix();
     renderer.setSize(width, height);
}

// --- Utility Functions ---
function formatMatrix(matrix) { /* ... Unchanged ... */
    const e = matrix.elements;
    let str = "";
    for (let i = 0; i < 4; i++) {
        str += `[ ${e[i].toFixed(3)}, ${e[i + 4].toFixed(3)}, ${e[i + 8].toFixed(3)}, ${e[i + 12].toFixed(3)} ]\n`;
    }
    return str.trim();
}
function generateDistinctColors(count) { /* ... Unchanged ... */
    const colors = [];
    const saturation = 0.6;
    const lightness = 0.5;
    for (let i = 0; i < count; i++) {
        const hue = (i * (360 / Math.max(1, count))) % 360 / 360;
        colors.push(new THREE.Color().setHSL(hue, saturation, lightness).getHex());
    }
    return colors;
}

// --- Start Execution Safely ---
if (document.readyState === 'loading') {  // Loading hasn't finished yet
    document.addEventListener('DOMContentLoaded', init);
} else {  // `DOMContentLoaded` has already fired
    init();
}

window.addEventListener('resize', onWindowResize);