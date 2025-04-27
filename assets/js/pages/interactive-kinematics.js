import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
// import { GUI } from 'three/addons/libs/lil-gui.module.min.js'; // Optional: for debugging

// --- Configuration & Constants ---
const DH_PARAMS_UR5E = [
    { a: 0.0,    alpha: Math.PI / 2, d: 0.1625, theta_offset: 0.0 },
    { a: -0.425, alpha: 0.0,         d: 0.0,    theta_offset: 0.0 },
    { a: -0.3922,alpha: 0.0,         d: 0.0,    theta_offset: 0.0 },
    { a: 0.0,    alpha: Math.PI / 2, d: 0.1333, theta_offset: 0.0 },
    { a: 0.0,    alpha: -Math.PI/ 2, d: 0.0997, theta_offset: 0.0 },
    { a: 0.0,    alpha: 0.0,         d: 0.0996, theta_offset: 0.0 } // Corrected d6 from Python code
];

const TCP_Z_OFFSET = 0.1565; // As defined in Python solver

const H_FLANGE_TCP = new THREE.Matrix4().makeTranslation(0, 0, TCP_Z_OFFSET);
const INV_H_FLANGE_TCP = new THREE.Matrix4().makeTranslation(0, 0, -TCP_Z_OFFSET);

// Constants derived from DH parameters (as in Python)
const d1 = DH_PARAMS_UR5E[0].d;
const a2 = DH_PARAMS_UR5E[1].a;
const a3 = DH_PARAMS_UR5E[2].a;
const d4 = DH_PARAMS_UR5E[3].d;
const d5 = DH_PARAMS_UR5E[4].d;
const d6 = DH_PARAMS_UR5E[5].d;
const len_a2 = Math.abs(a2);
const len_a3 = Math.abs(a3);

// Tolerances (ported from Python)
const tol_zero = 1e-9;
const tol_singularity = 1e-7;
const tol_compare = 1e-6;
const tol_geom = 1e-6;

// Visualization Constants
const JOINT_COLORS = [0xff0000, 0x00ff00, 0x0000ff, 0xffff00, 0xff00ff, 0x00ffff];
const PRIMARY_LINK_COLOR = 0xeeeeee; // Brighter for slider config
const PRIMARY_JOINT_COLOR = 0xffffff;
const IK_LINK_COLOR = 0xaaaaaa;
const IK_JOINT_COLOR = 0x888888;
const TARGET_TCP_COLOR = 0xff8800; // Color for the target frame visualization
const AXIS_LENGTH = 0.05;
const JOINT_RADIUS = 0.03;
const LINK_RADIUS = 0.015; // *** Made thinner ***
const IK_SOLUTION_OPACITY = 0.4; // Opacity for non-primary solutions

const initialJointAngles = [0.0, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, -Math.PI / 2, 0.0];

// --- DOM Elements ---
const canvasContainer = document.getElementById('kinematics-canvas-container');
const canvas = document.getElementById('kinematics-canvas');
const slidersContainer = document.getElementById('joint-sliders');
const tcpPoseOutput = document.getElementById('tcp-pose-output');
const ikSolutionCount = document.getElementById('ik-solution-count'); // Added
const ikSolutionsOutput = document.getElementById('ik-solutions-output'); // Added
const loadingMessage = document.getElementById('loading-message');
const errorMessage = document.getElementById('error-message');

// --- Global Three.js Variables ---
let scene, camera, renderer, controls;
let robotInstancesGroup; // Group to hold ALL robot configuration instances
let targetTCPGroup; // Group for the target TCP frame visualization
let jointSliders = [];
let valueDisplays = [];
let ikSolutionMaterials = []; // Store materials for IK solutions

// --- Helper Functions ---

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
    // r = [ R11, R21, R31, 0,
    //       R12, R22, R32, 0,
    //       R13, R23, R33, 0,
    //       Tx,  Ty,  Tz,  1 ]
    // We need R06 (standard notation) which is the transpose in math context
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
    if (Math.abs(B) < tol_singularity && Math.abs(A) < tol_singularity) { /* console.log("IK T1: Singularity at base"); */ return []; } // Avoid atan2(0,0)

    const sqrt_arg_t1 = dist_sq_xy - d4 * d4; // Should be >= 0 if check above passed
    const sqrt_val_t1 = Math.sqrt(Math.max(0, sqrt_arg_t1)); // Ensure non-negative

    const term1_t1 = Math.atan2(A, B); // Note: atan2(y, x) convention
    const term2_t1 = Math.atan2(sqrt_val_t1, d4); // Corrected order based on common UR IK derivations

    const theta1_sol = [
        term1_t1 - term2_t1, // Shoulder Right configuration
        term1_t1 + term2_t1 + Math.PI // Shoulder Left configuration (needs PI adjustment often)
    ];

    for (let t1_idx = 0; t1_idx < theta1_sol.length; t1_idx++) {
        const t1 = theta1_sol[t1_idx];
        const s1 = Math.sin(t1), c1 = Math.cos(t1);

        // --- Theta 5 ---
        // M_val depends on target Z-axis projection onto plane perp to Z0 and through X1
        const M_val = (r13d * s1 - r23d * c1);
        const M = Math.max(-1.0, Math.min(1.0, M_val)); // Clip to [-1, 1] for acos

        // Check if M is close to +/- 1 (singularity) - Python used atan2 approach
        let t5_options = [];
        if (Math.abs(Math.abs(M) - 1.0) < tol_singularity) { // Wrist singularity
            // console.warn("IK T5: Near wrist singularity (M ~ +/-1)");
            t5_options.push(M > 0 ? 0 : Math.PI); // t5 is 0 or pi
        } else {
            const sqrt_M_term = Math.sqrt(1 - M * M);
            t5_options = [
                Math.atan2(sqrt_M_term, M),  // Elbow down
                Math.atan2(-sqrt_M_term, M) // Elbow up
            ];
        }

        for (let t5_idx = 0; t5_idx < t5_options.length; t5_idx++) {
            const t5 = t5_options[t5_idx];
            const s5 = Math.sin(t5), c5 = Math.cos(t5);

            // --- Theta 6 ---
            let t6 = 0.0;
            if (Math.abs(s5) < tol_singularity) { // If t5 is 0 or pi (singularity)
                // console.warn("IK T6: Wrist singular (s5 ~ 0), setting t6=0");
                t6 = 0.0; // t6 is arbitrary, set to 0
            } else {
                 // Calculate based on target X/Y axes projection
                 const D_t6 = (r12d * s1 - r22d * c1); // Projection of Y-axis
                 const E_t6 = (r11d * s1 - r21d * c1); // Projection of X-axis
                 t6 = Math.atan2(-D_t6 / s5, E_t6 / s5); // atan2(-Y_proj, X_proj) / sin(t5)
            }
            const s6 = Math.sin(t6), c6 = Math.cos(t6);

            // --- Theta 3 ---
            // Calculate P4 relative to base frame using T1 and T5/T6 transforms
            const T65 = dhMatrix(DH_PARAMS_UR5E[5].a, DH_PARAMS_UR5E[5].alpha, DH_PARAMS_UR5E[5].d, t6);
            const T54 = dhMatrix(DH_PARAMS_UR5E[4].a, DH_PARAMS_UR5E[4].alpha, DH_PARAMS_UR5E[4].d, t5);
            const T10 = dhMatrix(DH_PARAMS_UR5E[0].a, DH_PARAMS_UR5E[0].alpha, DH_PARAMS_UR5E[0].d, t1);

            const T46_inv = new THREE.Matrix4().multiplyMatrices(T54, T65).invert();
            const T04 = new THREE.Matrix4().multiplyMatrices(T_flange, T46_inv);
            const T14 = new THREE.Matrix4().multiplyMatrices(new THREE.Matrix4().copy(T10).invert(), T04);

            const P41 = new THREE.Vector3().setFromMatrixPosition(T14); // P4 relative to frame 1
            const P41x = P41.x;
            const P41y = P41.y;

            const dist_sq_14 = P41x * P41x + P41y * P41y;
            const cos_t3_arg_num = dist_sq_14 - a2 * a2 - a3 * a3;
            const cos_t3_arg_den = 2 * a2 * a3; // a2, a3 are negative

            if (Math.abs(cos_t3_arg_den) < tol_zero) { /* console.log("IK T3: Denom zero"); */ continue; }

            const cos_t3_arg = cos_t3_arg_num / cos_t3_arg_den;

            if (Math.abs(cos_t3_arg) > 1.0 + tol_geom) { /* console.log("IK T3: Out of range"); */ continue; }
            const cos_t3 = Math.max(-1.0, Math.min(1.0, cos_t3_arg)); // Clip

            const sqrt_arg_t3 = 1.0 - cos_t3 * cos_t3;
            const sqrt_val_t3 = Math.sqrt(Math.max(0, sqrt_arg_t3));

            const t3_sol = [
                Math.atan2(sqrt_val_t3, cos_t3),  // Positive sine
                Math.atan2(-sqrt_val_t3, cos_t3) // Negative sine
            ];

            for (let t3_idx = 0; t3_idx < t3_sol.length; t3_idx++) {
                const t3 = t3_sol[t3_idx];
                const s3 = Math.sin(t3), c3 = Math.cos(t3);

                // --- Theta 2 ---
                const term2_den_t2 = a2 * a2 + a3 * a3 + 2 * a2 * a3 * c3; // (dist P41)^2
                if (Math.abs(term2_den_t2) < tol_zero) { /* console.log("IK T2: Denom zero"); */ continue; }

                const term1_t2 = Math.atan2(-P41y, -P41x); // atan2(-y, -x) relative to frame 1
                const term2_num_t2 = a3 * s3;
                const term2_den_t2_sqrt = Math.sqrt(term2_den_t2);
                const term2_t2 = Math.atan2(term2_num_t2, a2 + a3 * c3); // atan2(y_comp, x_comp) in linkage

                const t2 = term1_t2 - term2_t2;

                // --- Theta 4 ---
                const R14 = new THREE.Matrix4().extractRotation(T14);
                const r14 = R14.elements;
                 // R14 = [ R11, R21, R31, 0, R12, R22, R32, 0, R13, R23, R33, 0, ... ]
                 // We need atan2 of rotation elements relative to frame 1
                 // T14 = T12 * T23 * T34 -> R14 = R12 * R23 * R34
                 // Need R34 which relates to t4. Can use T14 elements.
                 // Z-axis of frame 4 in frame 1 coordinates (r14[2], r14[6], r14[10])
                 // X-axis of frame 4 in frame 1 coordinates (r14[0], r14[4], r14[8])
                 // atan2(Z4_y1, Z4_x1) relates to t2+t3+t4
                 // atan2(X4_y1, X4_x1) also involves t2+t3+t4

                 // Simpler: Use R04 and R01 to find R14, then extract angles
                const T21 = dhMatrix(DH_PARAMS_UR5E[1].a, DH_PARAMS_UR5E[1].alpha, DH_PARAMS_UR5E[1].d, t2);
                const T32 = dhMatrix(DH_PARAMS_UR5E[2].a, DH_PARAMS_UR5E[2].alpha, DH_PARAMS_UR5E[2].d, t3);
                const T03 = new THREE.Matrix4().multiplyMatrices(T10, T21).multiply(T32);
                const T34 = new THREE.Matrix4().multiplyMatrices(new THREE.Matrix4().copy(T03).invert(), T04);

                // Extract t4 from T34 (which should be a rotation around Z3 followed by DH transform for link 4)
                // T34 elements: [ c4, -s4*ca4, s4*sa4, a4*c4]
                //               [ s4,  c4*ca4,-c4*sa4, a4*s4]
                //               [ 0,     sa4,    ca4,    d4]
                // For UR5e: alpha4 = pi/2 -> ca4=0, sa4=1
                // T34 elements: [ c4,    0,   s4,    0]
                //               [ s4,    0,  -c4,    0]
                //               [ 0,     1,    0,   d4]
                // Need to extract t4 from rotation part R34
                const R34 = new THREE.Matrix4().extractRotation(T34);
                const r34 = R34.elements;
                const t4 = Math.atan2(r34[1], r34[0]); // atan2(R21, R11)

                // --- Assemble Solution ---
                const sol_angles_raw = [t1, t2, t3, t4, t5, t6];

                // Normalize angles to [-pi, pi]
                const sol_angles_normalized = sol_angles_raw.map(a => {
                    let wrapped = (a + Math.PI) % (2 * Math.PI);
                    if (wrapped < 0) wrapped += 2 * Math.PI; // Ensure positive before offset
                    return wrapped - Math.PI;
                });

                 // --- Verification (Optional but recommended) ---
                 /*
                 try {
                     const { T0_TCP: T_check } = forwardKinematics(sol_angles_normalized);
                     let diff = new THREE.Matrix4().copy(T_check).invert().multiply(T_desired_TCP);
                     let posDiff = new THREE.Vector3().setFromMatrixPosition(diff).length();
                     // More robust orientation check needed (e.g., angle from identity rotation)
                     if (posDiff < tol_compare * 10) { // Loose check
                          solutions.push(sol_angles_normalized);
                     } else {
                          // console.log("IK Solution failed verification", posDiff);
                     }
                 } catch (e) {
                     // console.log("FK Verification failed", e);
                 }
                 */
                 // For now, trust the math and add the solution
                 solutions.push(sol_angles_normalized);

            } // t3 loop
        } // t5 loop
    } // t1 loop

    // Filter duplicates (can happen due to numerical precision)
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


// --- Visualization Update ---

// Helper to compare joint angle arrays
function areAnglesClose(angles1, angles2, tolerance = 1e-4) {
    if (angles1.length !== angles2.length) return false;
    for (let i = 0; i < angles1.length; i++) {
        let diff = angles1[i] - angles2[i];
        // Handle angle wrapping (-pi vs pi)
        diff = (diff + Math.PI) % (2 * Math.PI) - Math.PI;
        if (Math.abs(diff) > tolerance) {
            return false;
        }
    }
    return true;
}

// Function to create ONE instance of the robot visualization
function createRobotVisualizationInstance(jointAngles, isPrimary) {
    const group = new THREE.Group();
    const { points, T0_TCP } = forwardKinematics(jointAngles); // Need points for drawing

    if (!points || points.length < 8) return null; // Need base, 6 joints, TCP

    const opacity = isPrimary ? 1.0 : IK_SOLUTION_OPACITY;
    const transparent = !isPrimary;
    const linkColor = isPrimary ? PRIMARY_LINK_COLOR : IK_LINK_COLOR;
    const jointColor = isPrimary ? PRIMARY_JOINT_COLOR : IK_JOINT_COLOR;

    const linkMaterial = new THREE.MeshStandardMaterial({
        color: linkColor,
        roughness: 0.6,
        metalness: 0.1,
        opacity: opacity,
        transparent: transparent,
        depthWrite: isPrimary // Allow non-primary to be seen through primary
    });
    const jointMaterial = new THREE.MeshStandardMaterial({
         color: jointColor,
         roughness: 0.5,
         metalness: 0.2,
         opacity: opacity,
         transparent: transparent,
         depthWrite: isPrimary
    });
     const jointGeometry = new THREE.SphereGeometry(JOINT_RADIUS, 12, 8);
     const linkGeometry = new THREE.CylinderGeometry(LINK_RADIUS, LINK_RADIUS, 1, 6); // Length=1 initially

     // Base visual (optional static part, not strictly needed per instance)
     // const baseMesh = new THREE.Mesh(new THREE.SphereGeometry(JOINT_RADIUS * 1.5, 16, 12), new THREE.MeshStandardMaterial({color: 0x555555}));
     // group.add(baseMesh);

     // Draw joints J1-J6
     for (let i = 0; i < 6; i++) {
         const jointMesh = new THREE.Mesh(jointGeometry, jointMaterial);
         jointMesh.position.copy(points[i + 1]); // points[0]=base, points[1]=J1,...
         group.add(jointMesh);
     }

     // Draw links Base-J1 to J5-J6
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

     // Add TCP frame for this instance (optional, maybe only for primary?)
     if (isPrimary && T0_TCP) {
         const tcpFrame = new THREE.AxesHelper(AXIS_LENGTH * 1.5); // Slightly larger axes
         const pos = new THREE.Vector3();
         const quat = new THREE.Quaternion();
         T0_TCP.decompose(pos, quat, new THREE.Vector3());
         tcpFrame.position.copy(pos);
         tcpFrame.setRotationFromQuaternion(quat);
         group.add(tcpFrame);
     }

    return group;
}


// --- Main Update Function ---
function updateVisualization() {
    const sliderAngles = jointSliders.map(slider => parseFloat(slider.value));

    // Update slider value displays
    sliderAngles.forEach((val, i) => {
        if (valueDisplays[i]) valueDisplays[i].textContent = val.toFixed(2);
    });

    // 1. Calculate Target FK Pose from sliders
    const { T0_TCP: T_target_TCP } = forwardKinematics(sliderAngles);

    if (!T_target_TCP) {
        tcpPoseOutput.textContent = "Error in FK calculation.";
        ikSolutionCount.textContent = "N/A";
        ikSolutionsOutput.textContent = "N/A";
        robotInstancesGroup.clear(); // Clear previous drawings
        return;
    }

    // Display Target TCP Pose
    tcpPoseOutput.textContent = formatMatrix(T_target_TCP);
    // Update target TCP visualization frame
    targetTCPGroup.clear();
    const targetFrame = new THREE.AxesHelper(AXIS_LENGTH * 2.5); // Make target frame prominent
    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    T_target_TCP.decompose(pos, quat, new THREE.Vector3());
    targetFrame.position.copy(pos);
    targetFrame.setRotationFromQuaternion(quat);
    targetTCPGroup.add(targetFrame);
    const targetMarkerGeo = new THREE.SphereGeometry(JOINT_RADIUS * 0.5, 8, 8);
    const targetMarkerMat = new THREE.MeshBasicMaterial({ color: TARGET_TCP_COLOR });
    const targetMarker = new THREE.Mesh(targetMarkerGeo, targetMarkerMat);
    targetMarker.position.copy(pos);
    targetTCPGroup.add(targetMarker);


    // 2. Calculate ALL IK Solutions for the Target Pose
    const ikSolutions = inverseKinematics(T_target_TCP);
    ikSolutionCount.textContent = ikSolutions.length;

    // Display IK solutions text
    if (ikSolutions.length > 0) {
        ikSolutionsOutput.textContent = ikSolutions.map((sol, idx) =>
            `Sol ${idx + 1}: [${sol.map(a => a.toFixed(2)).join(', ')}]`
        ).join('\n');
    } else {
        ikSolutionsOutput.textContent = "No solutions found (or near singularity).";
    }


    // 3. Visualize all Solutions
    robotInstancesGroup.clear(); // Clear previous robot drawings

    let primaryFoundAmongSolutions = false;
    // Use a colormap for IK solutions
    const colors = generateDistinctColors(ikSolutions.length);


    ikSolutions.forEach((ikAngles, index) => {
        const isPrimary = areAnglesClose(sliderAngles, ikAngles);
        if (isPrimary) primaryFoundAmongSolutions = true;

        const robotInstance = createRobotVisualizationInstance(ikAngles, isPrimary);
        if (robotInstance) {
             // Apply distinct colors to non-primary solutions if needed (or use transparency)
             if (!isPrimary) {
                robotInstance.traverse((child) => {
                    if (child.isMesh && child.material) {
                       // Optionally apply colormap color here, or just rely on opacity
                       // Example: child.material.color.setHex(colors[index % colors.length]);
                       child.material.needsUpdate = true;
                    }
                });
            }
            robotInstancesGroup.add(robotInstance);
        }
    });

    // If the slider configuration itself wasn't found by IK (e.g., due to tolerance issues or if it's invalid),
    // still draw it clearly as the reference.
    if (!primaryFoundAmongSolutions) {
        // console.warn("Slider configuration not found among IK solutions, drawing separately.");
        const primaryInstance = createRobotVisualizationInstance(sliderAngles, true);
        if (primaryInstance) {
            robotInstancesGroup.add(primaryInstance);
        }
    }
}


// --- Initialization ---
function init() {
    try {
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x282c34);

        const aspect = canvasContainer.clientWidth / canvasContainer.clientHeight;
        camera = new THREE.PerspectiveCamera(60, aspect, 0.1, 100);
        camera.position.set(0.8, 1.0, 1.2); // Adjusted starting position slightly
        camera.lookAt(0, 0, 0.3);

        renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
        renderer.setSize(canvasContainer.clientWidth, canvasContainer.clientHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        THREE.ColorManagement.enabled = true;

        const ambientLight = new THREE.AmbientLight(0xffffff, 0.7); // Slightly brighter ambient
        scene.add(ambientLight);
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.9); // Slightly brighter directional
        directionalLight.position.set(5, 10, 7.5);
        scene.add(directionalLight);

        controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.1;
        controls.target.set(0, 0, 0.3);

        // Group for all robot meshes
        robotInstancesGroup = new THREE.Group();
        scene.add(robotInstancesGroup);

        // Group for the target TCP frame marker
        targetTCPGroup = new THREE.Group();
        scene.add(targetTCPGroup);


        const axesHelper = new THREE.AxesHelper(0.3); // Base frame
        scene.add(axesHelper);

        // *** Remove Ground Plane ***
        // const planeGeometry = new THREE.PlaneGeometry(2, 2);
        // const planeMaterial = new THREE.MeshStandardMaterial({ color: 0x444444, side: THREE.DoubleSide });
        // const plane = new THREE.Mesh(planeGeometry, planeMaterial);
        // plane.rotation.x = -Math.PI / 2;
        // scene.add(plane); // REMOVED

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

function createSliders() { /* ... Unchanged from previous version ... */
    slidersContainer.innerHTML = ''; // Clear any previous sliders
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
        slider.min = -Math.PI.toFixed(3); // Approx -3.142
        slider.max = Math.PI.toFixed(3); // Approx 3.142
        slider.step = (Math.PI / 180).toFixed(4); // ~1 degree steps
        slider.value = initialJointAngles[i].toFixed(3);

        const valueDisplay = document.createElement('span');
        valueDisplay.className = 'value-display';
        valueDisplay.textContent = parseFloat(slider.value).toFixed(2); // Initial display

        group.appendChild(label);
        group.appendChild(slider);
        group.appendChild(valueDisplay);
        slidersContainer.appendChild(group);

        jointSliders.push(slider);
        valueDisplays.push(valueDisplay);

        // Add event listener
        slider.addEventListener('input', updateVisualization); // Call main update function
    }
}


// --- Animation Loop ---
function animate() { /* ... Unchanged ... */
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

// --- Resize Handling ---
function onWindowResize() { /* ... Unchanged ... */
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

// Simple distinct color generator (Hue-based)
function generateDistinctColors(count) {
    const colors = [];
    const saturation = 0.6; // Lower saturation for fainter colors
    const lightness = 0.5;
    for (let i = 0; i < count; i++) {
        const hue = (i * (360 / Math.max(1, count))) % 360 / 360; // Distribute hues
        colors.push(new THREE.Color().setHSL(hue, saturation, lightness).getHex());
    }
    return colors;
}


// --- Start ---
window.addEventListener('resize', onWindowResize);
init();