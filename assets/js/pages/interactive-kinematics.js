import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// --- UR5e Constants ---
const DH_PARAMS_UR5E = [
    { a: 0.0,    alpha: Math.PI / 2, d: 0.1625, theta_offset: 0.0 },
    { a: -0.425, alpha: 0.0,         d: 0.0,    theta_offset: 0.0 },
    { a: -0.3922,alpha: 0.0,         d: 0.0,    theta_offset: 0.0 },
    { a: 0.0,    alpha: Math.PI / 2, d: 0.1333, theta_offset: 0.0 },
    { a: 0.0,    alpha: -Math.PI / 2,d: 0.0997, theta_offset: 0.0 },
    { a: 0.0,    alpha: 0.0,         d: 0.0996, theta_offset: 0.0 } // Corrected d6 based on Python code
];

const TCP_Z_OFFSET = 0.1565; // Example TCP offset, adjust if needed

// Transformation from Flange (Joint 6) to TCP
const H_FLANGE_TCP = new THREE.Matrix4().makeTranslation(0, 0, TCP_Z_OFFSET);
// Inverse Transformation
const INV_H_FLANGE_TCP = new THREE.Matrix4().copy(H_FLANGE_TCP).invert();

// Extract DH parameters for easier access in IK
const d1 = DH_PARAMS_UR5E[0].d;
const a2 = DH_PARAMS_UR5E[1].a;
const a3 = DH_PARAMS_UR5E[2].a;
const d4 = DH_PARAMS_UR5E[3].d;
const d5 = DH_PARAMS_UR5E[4].d;
const d6 = DH_PARAMS_UR5E[5].d;
const len_a2 = Math.abs(a2);
const len_a3 = Math.abs(a3);

// Tolerances (from Python)
const tol_zero = 1e-9;
const tol_singularity = 1e-7;
const tol_compare = 1e-5; // Slightly looser for JS floating point
const tol_geom = 1e-6;

// --- Utility Functions ---

/**
 * Clamps a value between a minimum and maximum.
 * @param {number} value The value to clamp.
 * @param {number} min The minimum allowed value.
 * @param {number} max The maximum allowed value.
 * @returns {number} The clamped value.
 */
function clamp(value, min, max) {
    return Math.max(min, Math.min(value, max));
}

/**
 * Normalizes an angle to the range [-PI, PI].
 * @param {number} angle Angle in radians.
 * @returns {number} Normalized angle in radians.
 */
function normalizeAngle(angle) {
    while (angle <= -Math.PI) angle += 2 * Math.PI;
    while (angle > Math.PI) angle -= 2 * Math.PI;
    return angle;
}

/**
 * Creates a Denavit-Hartenberg transformation matrix.
 * @param {number} a Link length.
 * @param {number} alpha Link twist.
 * @param {number} d Link offset.
 * @param {number} theta Joint angle.
 * @returns {THREE.Matrix4} The transformation matrix.
 */
function dhMatrix(a, alpha, d, theta) {
    const cos_t = Math.cos(theta);
    const sin_t = Math.sin(theta);
    const cos_a = Math.cos(alpha);
    const sin_a = Math.sin(alpha);

    const matrix = new THREE.Matrix4();
    matrix.set(
        cos_t, -sin_t * cos_a,  sin_t * sin_a, a * cos_t,
        sin_t,  cos_t * cos_a, -cos_t * sin_a, a * sin_t,
            0,          sin_a,          cos_a,         d,
            0,              0,              0,         1
    );
    return matrix;
}

// --- Kinematics Functions ---

/**
 * Calculates Forward Kinematics for the UR5e.
 * @param {number[]} jointAngles Array of 6 joint angles in radians.
 * @param {object[]} dhParams DH parameters array.
 * @param {THREE.Matrix4} HFlangeTcp Transformation from flange to TCP.
 * @returns {{ points: THREE.Vector3[], T0_TCP: THREE.Matrix4, T0_Flange: THREE.Matrix4 } | null} Object containing joint positions, TCP pose, and Flange pose, or null on error.
 */
function forwardKinematics(jointAngles, dhParams = DH_PARAMS_UR5E, HFlangeTcp = H_FLANGE_TCP) {
    if (jointAngles.length !== dhParams.length) {
        console.error(`FK Error: Angle count mismatch. Expected ${dhParams.length}, got ${jointAngles.length}`);
        return null;
    }

    const transforms = [new THREE.Matrix4()]; // Base frame (identity)
    const points = [new THREE.Vector3(0, 0, 0)]; // Base origin
    let T_prev = transforms[0].clone();

    for (let i = 0; i < dhParams.length; i++) {
        const p = dhParams[i];
        const T_i_minus_1_to_i = dhMatrix(p.a, p.alpha, p.d, jointAngles[i] + p.theta_offset);
        const T_curr = new THREE.Matrix4().multiplyMatrices(T_prev, T_i_minus_1_to_i);

        transforms.push(T_curr);
        const position = new THREE.Vector3().setFromMatrixPosition(T_curr);
        points.push(position);
        T_prev = T_curr.clone();
    }

    const T0_Flange = transforms[transforms.length - 1];
    const T0_TCP = new THREE.Matrix4().multiplyMatrices(T0_Flange, HFlangeTcp);
    points.push(new THREE.Vector3().setFromMatrixPosition(T0_TCP)); // Add TCP point

    return { points, T0_TCP, T0_Flange };
}


/**
 * Calculates Inverse Kinematics for the UR5e.
 * Adapted from the provided Python analytical solver.
 * @param {THREE.Matrix4} T_desired_TCP The desired TCP pose matrix relative to the base.
 * @param {object[]} dhParams DH parameters array.
 * @param {THREE.Matrix4} invHFlangeTcp Inverse transformation from TCP to flange.
 * @returns {Array<[number[], number, number, number]>} Array of solutions. Each solution is [angles[6], t1_idx, t5_idx, t3_idx]. Returns empty array if no solutions found.
 */
function inverseKinematics(T_desired_TCP, dhParams = DH_PARAMS_UR5E, invHFlangeTcp = INV_H_FLANGE_TCP) {
    const solutions = [];
    const T_flange = new THREE.Matrix4().multiplyMatrices(T_desired_TCP, invHFlangeTcp);

    // Extract position and rotation from the desired Flange pose
    const P60 = new THREE.Vector3().setFromMatrixPosition(T_flange); // Position of wrist 3 (flange) origin
    const R06 = new THREE.Matrix3().setFromMatrix4(T_flange);       // Rotation matrix from base to flange

    const pxd = P60.x;
    const pyd = P60.y;
    const pzd = P60.z;

    const r11d = R06.elements[0]; const r12d = R06.elements[3]; const r13d = R06.elements[6];
    const r21d = R06.elements[1]; const r22d = R06.elements[4]; const r23d = R06.elements[7];
    const r31d = R06.elements[2]; const r32d = R06.elements[5]; const r33d = R06.elements[8];

    // --- Calculate Theta 1 ---
    // Position of wrist 2 center (P50) projected onto the XY plane
    const P50 = new THREE.Vector3(
        pxd - d6 * r13d,
        pyd - d6 * r23d,
        pzd - d6 * r33d
    );
    const P50x = P50.x;
    const P50y = P50.y;
    // const P50z = P50.z; // Needed later for theta 2, 3

    // Check reachability for theta1
    const dist_sq_xy = P50x ** 2 + P50y ** 2;
    const sqrt_arg_t1_sq = dist_sq_xy - d4 ** 2;

    if (sqrt_arg_t1_sq < -tol_geom) { // Use negative tolerance to allow for floating point errors near zero
        // console.warn("IK Warn: Position potentially out of reach for theta1 (sqrt_arg < 0).", sqrt_arg_t1_sq);
        return []; // Out of reach
    }
    const sqrt_arg_t1 = Math.max(0, sqrt_arg_t1_sq); // Clamp to zero if slightly negative due to precision
    const sqrt_val_t1 = Math.sqrt(sqrt_arg_t1);

    // Check singularity for theta1 (P50 projection close to origin)
    if (Math.abs(P50x) < tol_singularity && Math.abs(P50y) < tol_singularity) {
       // console.warn("IK Warn: Singularity detected for theta1 (P50x and P50y near zero).");
       // At this singularity, theta1 is undefined. We might need special handling,
       // but for now, return no solutions as per the original logic.
       return [];
    }


    // Two possible solutions for theta1
    // Note: Python atan2(y, x), JS Math.atan2(y, x)
    const term1_t1 = Math.atan2(P50y, P50x); // atan2 of the wrist center projection
    const term2_t1 = Math.atan2(d4, sqrt_val_t1); // Angle related to d4 offset

    const theta1_sol = [
        normalizeAngle(term1_t1 - term2_t1 + Math.PI / 2), // Shoulder right
        normalizeAngle(term1_t1 + term2_t1 - Math.PI / 2)  // Shoulder left
    ];


    // --- Iterate through Theta 1 solutions ---
    for (let t1_idx = 0; t1_idx < theta1_sol.length; t1_idx++) {
        const t1 = theta1_sol[t1_idx];
        const s1 = Math.sin(t1);
        const c1 = Math.cos(t1);

        // --- Calculate Theta 5 ---
        // M depends on the Z-axis of the desired flange orientation projected onto the XY plane after rotating by -t1
        const M_val = r13d * s1 - r23d * c1;
        const M = clamp(M_val, -1.0, 1.0); // Ensure M is within [-1, 1] for acos/atan2 later

        // Check for singularity (wrist singularity: when TCP Z-axis aligns with J5 axis)
        // This happens when M is close to +/- 1.
        const sqrt_arg_t5_sq = 1.0 - M * M;
        if (sqrt_arg_t5_sq < -tol_geom) {
             // Should not happen if M is clamped, but check anyway
             // console.warn(`IK Warn (t1_idx ${t1_idx}): Invalid state for theta5 calculation (sqrt_arg < 0). M=${M}`);
             continue;
        }
        const sqrt_arg_t5 = Math.max(0, sqrt_arg_t5_sq);
        const sqrt_val_t5 = Math.sqrt(sqrt_arg_t5); // This is |sin(t5)|

        // Two possible solutions for theta5 (elbow up/down relative to wrist)
        const t5_sol = [
            normalizeAngle(Math.atan2(sqrt_val_t5, M)),  // Elbow down
            normalizeAngle(Math.atan2(-sqrt_val_t5, M)) // Elbow up
        ];


        // --- Iterate through Theta 5 solutions ---
        for (let t5_idx = 0; t5_idx < t5_sol.length; t5_idx++) {
            const t5 = t5_sol[t5_idx];
            const s5 = Math.sin(t5);
            const c5 = Math.cos(t5);

            let t6 = 0.0; // Default t6

            // Check for wrist singularity (t5 near 0 or PI)
            const is_singular_s5 = Math.abs(s5) < tol_singularity;

            if (is_singular_s5) {
                // console.warn(`IK Warn (t1_idx ${t1_idx}, t5_idx ${t5_idx}): Wrist singularity detected (t5 near 0 or PI). Setting t6=0.`);
                // At singularity, t1+t6 is coupled. We can often set t6=0 and solve for t1.
                // The original code sets t6 = 0.0 here. Let's stick to that.
                t6 = 0.0;
            } else {
                // Calculate Theta 6 using the X and Y axes of the desired flange orientation
                // D and E relate to the projection of the desired flange X/Y axes onto the robot's frame rotated by t1
                const D = - (r12d * s1 - r22d * c1); // Corresponds to Y-axis projection / s5
                const E = - (r11d * s1 - r21d * c1); // Corresponds to X-axis projection / s5

                if (Math.abs(D) < tol_singularity && Math.abs(E) < tol_singularity) {
                     // This case might indicate an issue or another singularity type.
                     // console.warn(`IK Warn (t1_idx ${t1_idx}, t5_idx ${t5_idx}): D and E near zero in t6 calculation.`);
                     continue; // Skip this t5 solution if X/Y axes info is lost
                }
                t6 = normalizeAngle(Math.atan2(D / s5, E / s5)); // atan2(y, x)
            }

            // --- Calculate Theta 3 ---
            // This involves the 'elbow' joint, using the distance between wrist 1 and wrist 2 centers.
            const s6 = Math.sin(t6);
            const c6 = Math.cos(t6);

            // Calculate t234 (sum angle) based on desired orientation and t1, t5, t6
            // Relates the Z-axis of frame 3 to the Z-axis of the base frame
            const atan_y_234 = -s5 * (r13d * c1 + r23d * s1); // Component related to base Z
            const atan_x_234 = r33d;                          // Component related to desired Z

             // Check singularity for t234 (when desired Z aligns with base Z?)
            if (Math.abs(atan_y_234) < tol_singularity && Math.abs(atan_x_234) < tol_singularity) {
                // console.warn(`IK Warn (t1_idx ${t1_idx}, t5_idx ${t5_idx}): atan_y/x_234 near zero.`);
                // This might indicate alignment issues or singularities.
                continue;
            }
            const t234 = Math.atan2(atan_y_234, atan_x_234);
            const s234 = Math.sin(t234);
            const c234 = Math.cos(t234);


            // Calculate position of wrist 1 center (P30) projected onto the plane defined by J2 and J3 axes
            // KC: Projection along the axis from J1 to P50 (in the rotated frame)
            // KS: Projection along the Z-axis (vertical)
            const KC = P50x * c1 + P50y * s1 - (s234 * d5 + c234 * c5 * d6); // Corrected based on common UR IK derivations (sign of d6 term depends on convention) - Let's re-verify
            const KS = P50.z - d1 + (c234 * d5 - s234 * c5 * d6); // Corrected based on common UR IK derivations

            // Use Law of Cosines on the triangle formed by J1, J2, J3 projected onto the plane
            const dist_sq_13 = KC * KC + KS * KS; // Squared distance between J1 and J3 projected centers
            const denom_t3 = 2 * len_a2 * len_a3;

            if (Math.abs(denom_t3) < tol_zero) {
                // console.warn(`IK Warn (t1_idx ${t1_idx}, t5_idx ${t5_idx}): Denominator for t3 is zero (a2 or a3 is zero).`);
                continue; // Should not happen for UR5e
            }

            const cos_t3_arg = (dist_sq_13 - len_a2 * len_a2 - len_a3 * len_a3) / denom_t3;

            // Check reachability for the elbow joint
            if (Math.abs(cos_t3_arg) > 1.0 + tol_geom) {
                // console.warn(`IK Warn (t1_idx ${t1_idx}, t5_idx ${t5_idx}): Elbow out of reach |cos(t3)| > 1. Arg: ${cos_t3_arg}`);
                continue; // Cannot form the triangle
            }
            const cos_t3 = clamp(cos_t3_arg, -1.0, 1.0); // Clamp for safety

            const sqrt_arg_t3_sq = 1.0 - cos_t3 * cos_t3;
             if (sqrt_arg_t3_sq < -tol_geom) {
                 // Should not happen if clamped, but check
                 // console.warn(`IK Warn (t1_idx ${t1_idx}, t5_idx ${t5_idx}): Invalid state for theta3 calculation (sqrt_arg < 0). cos_t3=${cos_t3}`);
                 continue;
             }
            const sqrt_arg_t3 = Math.max(0, sqrt_arg_t3_sq);
            const sqrt_val_t3 = Math.sqrt(sqrt_arg_t3); // This is |sin(t3)|

            // Two possible solutions for theta3 (elbow up/down relative to shoulder)
            const t3_sol = [
                normalizeAngle(Math.atan2(sqrt_val_t3, cos_t3)), // Elbow down
                normalizeAngle(Math.atan2(-sqrt_val_t3, cos_t3)) // Elbow up
            ];

            // --- Iterate through Theta 3 solutions ---
            for (let t3_idx = 0; t3_idx < t3_sol.length; t3_idx++) {
                const t3 = t3_sol[t3_idx];
                const s3 = Math.sin(t3);
                const c3 = cos_t3; // Use the calculated cosine directly

                // --- Calculate Theta 2 ---
                // Uses the triangle J1-J2-J3 again
                const term1_t2_y = KS;
                const term1_t2_x = KC;

                 if (Math.abs(term1_t2_y) < tol_singularity && Math.abs(term1_t2_x) < tol_singularity) {
                    // console.warn(`IK Warn (t1..t3 idx ${t1_idx},${t5_idx},${t3_idx}): KS and KC near zero in t2 calculation.`);
                    continue; // Avoid atan2(0,0)
                 }
                const term1_t2 = Math.atan2(term1_t2_y, term1_t2_x); // Angle of the vector from J1 to J3 projected center

                const term2_t2_y = len_a3 * s3;
                const term2_t2_x = len_a2 + len_a3 * c3; // Note: a2 is negative, len_a2 is positive. Using len_a2 here. Check original derivation if issues.

                 if (Math.abs(term2_t2_y) < tol_singularity && Math.abs(term2_t2_x) < tol_singularity) {
                    // console.warn(`IK Warn (t1..t3 idx ${t1_idx},${t5_idx},${t3_idx}): term2 y/x near zero in t2 calculation.`);
                    continue; // Avoid atan2(0,0)
                 }
                const term2_t2 = Math.atan2(term2_t2_y, term2_t2_x); // Angle within the J1-J2-J3 triangle

                const t2 = normalizeAngle(term1_t2 - term2_t2);

                // --- Calculate Theta 4 ---
                const t4 = normalizeAngle(t234 - t2 - t3);

                // --- Store Solution ---
                const sol_angles_raw = [t1, t2, t3, t4, t5, t6];
                // Normalize all angles to [-pi, pi] for consistency
                const sol_angles_normalized = sol_angles_raw.map(normalizeAngle);

                // --- Verification (Optional but Recommended) ---
                // Perform FK check to ensure the solution reaches the target
                const fkCheck = forwardKinematics(sol_angles_normalized, dhParams, H_FLANGE_TCP);
                if (fkCheck) {
                    const T_check_TCP = fkCheck.T0_TCP;
                    // Compare matrices element-wise (or check position and quaternion difference)
                    let diff = 0;
                    for(let k=0; k<16; k++) {
                        diff += Math.abs(T_check_TCP.elements[k] - T_desired_TCP.elements[k]);
                    }

                    if (diff < tol_compare * 16) { // Check average element difference
                         solutions.push([sol_angles_normalized, t1_idx, t5_idx, t3_idx]);
                    } else {
                        // console.warn(`IK Verify Fail (t1..t3 idx ${t1_idx},${t5_idx},${t3_idx}): FK mismatch. Diff: ${diff.toFixed(5)}`);
                        // console.log("Target:", T_desired_TCP.elements.map(n=>n.toFixed(3)));
                        // console.log("Actual:", T_check_TCP.elements.map(n=>n.toFixed(3)));
                        // console.log("Angles:", sol_angles_normalized.map(a => (a * 180 / Math.PI).toFixed(1)));
                    }
                } else {
                     console.error("IK Verify Error: FK failed during verification.");
                }

            } // end t3 loop
        } // end t5 loop
    } // end t1 loop

    // Remove duplicate solutions (can happen near singularities)
    const uniqueSolutions = [];
    const seenSolutions = new Set();
    for (const sol of solutions) {
        const key = sol[0].map(a => a.toFixed(4)).join(','); // Key based on rounded angles
        if (!seenSolutions.has(key)) {
            uniqueSolutions.push(sol);
            seenSolutions.add(key);
        }
    }

    return uniqueSolutions;
}


// --- Three.js Setup ---
let scene, camera, renderer, controls;
let robotGroup; // Group to hold all robot visualizations
let targetFrameHelper; // To show the FK target pose
const sliderJointAngles = [0, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, -Math.PI / 2, 0]; // Initial angles
const sliders = [];
const sliderValueLabels = [];

const canvas = document.getElementById('robot-canvas');
const canvasContainer = document.getElementById('canvas-container');
const slidersContainer = document.getElementById('sliders-container');
const infoDisplay = document.getElementById('info-display');
const loadingIndicator = document.getElementById('loading-indicator');

/** Initialize the 3D scene, camera, renderer, and controls */
function initThreeJS() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a202c); // Match body background

    // Camera
    const aspect = canvas.clientWidth / canvas.clientHeight;
    camera = new THREE.PerspectiveCamera(50, aspect, 0.1, 100);
    camera.position.set(1.5, 1.5, 1.5); // Adjusted initial camera position
    camera.lookAt(0, 0.3, 0); // Look towards the robot base area

    // Renderer
    renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(canvas.clientWidth, canvas.clientHeight); // Initial size

    // Lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 10, 7);
    scene.add(directionalLight);

    // Controls
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.1;
    controls.screenSpacePanning = false;
    controls.target.set(0, 0.3, 0); // Set control target near robot base
    controls.update();

    // Robot Group
    robotGroup = new THREE.Group();
    scene.add(robotGroup);

    // Base Helper
    const gridHelper = new THREE.GridHelper(2, 10, 0x444444, 0x888888);
    gridHelper.position.y = -0.01; // Slightly below base
    scene.add(gridHelper);
    const axesHelper = new THREE.AxesHelper(0.2); // Base frame
    scene.add(axesHelper);

    // Target Frame Helper (initially hidden)
    targetFrameHelper = new THREE.AxesHelper(0.15); // Larger target frame
    targetFrameHelper.visible = false;
    scene.add(targetFrameHelper);

    // Handle window resize
    window.addEventListener('resize', onWindowResize);
    onWindowResize(); // Call once initially

    // Start animation loop
    animate();

    console.log("Three.js initialized.");
    loadingIndicator.style.display = 'none'; // Hide loading indicator
}

/** Handle window resize events */
function onWindowResize() {
    const containerWidth = canvasContainer.clientWidth;
    // Make height proportional to width, e.g., 3:4 aspect ratio, or fixed
    const containerHeight = Math.min(500, containerWidth * 0.75); // Max height 500px

    canvas.style.height = `${containerHeight}px`; // Update style height

    camera.aspect = containerWidth / containerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(containerWidth, containerHeight);
}

/** Animation loop */
function animate() {
    requestAnimationFrame(animate);
    controls.update(); // Required if enableDamping is true
    renderer.render(scene, camera);
}

// --- Robot Drawing Functions ---

const jointRadius = 0.03;
const linkRadius = 0.02;
const jointColor = 0xffffff; // White joints
const linkColor = 0x777777; // Grey links
const tcpColor = 0xffaa00; // Orange TCP sphere

const fkConfigColor = 0xff0000; // Red for FK config
const ikConfigColor = 0x0077ff; // Blue for IK solutions

const jointMaterial = new THREE.MeshStandardMaterial({ color: jointColor, metalness: 0.5, roughness: 0.5 });
const linkMaterial = new THREE.MeshStandardMaterial({ color: linkColor, metalness: 0.5, roughness: 0.5 });
const tcpMaterial = new THREE.MeshStandardMaterial({ color: tcpColor, metalness: 0.5, roughness: 0.5 });

const fkLinkMaterial = new THREE.MeshStandardMaterial({ color: fkConfigColor, metalness: 0.5, roughness: 0.5, transparent: true, opacity: 0.8 });
const fkJointMaterial = new THREE.MeshStandardMaterial({ color: fkConfigColor, metalness: 0.5, roughness: 0.5, transparent: true, opacity: 0.8 });

const ikLinkMaterial = new THREE.MeshStandardMaterial({ color: ikConfigColor, metalness: 0.5, roughness: 0.5, transparent: true, opacity: 0.3 });
const ikJointMaterial = new THREE.MeshStandardMaterial({ color: ikConfigColor, metalness: 0.5, roughness: 0.5, transparent: true, opacity: 0.3 });


/**
 * Creates and adds a robot arm visualization to the scene.
 * @param {THREE.Vector3[]} points Array of joint positions (including base and TCP).
 * @param {THREE.Matrix4} T0_TCP The TCP pose matrix.
 * @param {string} type 'fk' or 'ik' to determine materials.
 */
function drawRobot(points, T0_TCP, type = 'ik') {
    if (!points || points.length < 8) { // Expect Base + 6 Joints + TCP = 8 points
        console.error("Draw Error: Insufficient points provided.", points);
        return;
    }

    const group = new THREE.Group();
    const currentLinkMat = (type === 'fk') ? fkLinkMaterial : ikLinkMaterial;
    const currentJointMat = (type === 'fk') ? fkJointMaterial : jointMaterial; // FK joints solid red

    // Draw Links (as cylinders)
    for (let i = 0; i < points.length - 1; i++) {
        const startPoint = points[i];
        const endPoint = points[i + 1];
        const distance = startPoint.distanceTo(endPoint);

        if (distance < 1e-4) continue; // Skip zero-length links

        const cylinderGeo = new THREE.CylinderGeometry(linkRadius, linkRadius, distance, 16);
        // By default, cylinder points up Y axis. We need to rotate it.
        const cylinder = new THREE.Mesh(cylinderGeo, currentLinkMat);

        // Position cylinder halfway between points
        cylinder.position.copy(startPoint).add(endPoint).multiplyScalar(0.5);

        // Make cylinder point from startPoint to endPoint
        const direction = new THREE.Vector3().subVectors(endPoint, startPoint).normalize();
        const quaternion = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction);
        cylinder.setRotationFromQuaternion(quaternion);

        group.add(cylinder);
    }

    // Draw Joints (as spheres) - Skip base (point 0)
    const jointGeo = new THREE.SphereGeometry(jointRadius, 16, 16);
    for (let i = 1; i < points.length -1; i++) { // Exclude base and TCP point for generic joints
         const jointSphere = new THREE.Mesh(jointGeo, currentJointMat);
         jointSphere.position.copy(points[i]);
         group.add(jointSphere);
    }

    // Draw TCP (as a slightly larger sphere)
    const tcpGeo = new THREE.SphereGeometry(jointRadius * 1.2, 16, 16);
    const tcpSphere = new THREE.Mesh(tcpGeo, tcpMaterial);
    tcpSphere.position.copy(points[points.length - 1]); // Last point is TCP
    group.add(tcpSphere);

    // Draw TCP Frame (small axes helper attached to TCP)
    const tcpFrame = new THREE.AxesHelper(0.08);
    tcpFrame.position.copy(points[points.length - 1]);
    // Apply rotation from T0_TCP
    const tcpRotation = new THREE.Quaternion().setFromRotationMatrix(T0_TCP);
    tcpFrame.setRotationFromQuaternion(tcpRotation);
    group.add(tcpFrame);


    robotGroup.add(group); // Add this robot configuration to the main group
}

/** Clear all previously drawn robot arms */
function clearRobots() {
    // Dispose geometries and materials to free memory (optional but good practice)
    robotGroup.traverse(child => {
        if (child instanceof THREE.Mesh) {
            if (child.geometry) child.geometry.dispose();
            // Don't dispose shared materials like ikLinkMaterial etc.
            // If materials were created uniquely per robot, dispose them here.
        }
    });
    // Remove all children from the group
    robotGroup.clear();
}

// --- UI Update Functions ---

/** Create sliders for joint control */
function createSliders() {
    slidersContainer.innerHTML = ''; // Clear placeholder
    for (let i = 0; i < 6; i++) {
        const container = document.createElement('div');
        container.className = 'slider-container';

        const label = document.createElement('label');
        label.className = 'slider-label';
        label.htmlFor = `joint${i}-slider`;
        label.textContent = `J${i + 1}:`;

        const slider = document.createElement('input');
        slider.type = 'range';
        slider.id = `joint${i}-slider`;
        slider.className = 'slider';
        slider.min = -Math.PI.toFixed(3);
        slider.max = Math.PI.toFixed(3);
        slider.step = (Math.PI / 180).toFixed(3); // 1 degree steps
        slider.value = sliderJointAngles[i].toFixed(3);
        slider.addEventListener('input', onSliderChange); // Use 'input' for real-time update

        const valueLabel = document.createElement('span');
        valueLabel.className = 'slider-value';
        valueLabel.id = `joint${i}-value`;
        valueLabel.textContent = `${(sliderJointAngles[i] * 180 / Math.PI).toFixed(1)}°`;

        container.appendChild(label);
        container.appendChild(slider);
        container.appendChild(valueLabel);
        slidersContainer.appendChild(container);

        sliders.push(slider);
        sliderValueLabels.push(valueLabel);
    }
    console.log("Sliders created.");
}

/** Handle slider changes */
function onSliderChange() {
    for (let i = 0; i < 6; i++) {
        sliderJointAngles[i] = parseFloat(sliders[i].value);
        sliderValueLabels[i].textContent = `${(sliderJointAngles[i] * 180 / Math.PI).toFixed(1)}°`;
    }
    updateVisualization(); // Update the 3D view and IK
}

/** Update the 3D visualization and IK calculations */
function updateVisualization() {
    clearRobots(); // Remove previous arms

    // 1. Forward Kinematics for Slider Angles
    const fkResult = forwardKinematics(sliderJointAngles);
    let T_Target_TCP = null;
    let fkSuccess = false;

    if (fkResult) {
        T_Target_TCP = fkResult.T0_TCP;
        // Draw the FK robot configuration (red)
        drawRobot(fkResult.points, T_Target_TCP, 'fk');
        // Update and show the target frame helper
        targetFrameHelper.position.setFromMatrixPosition(T_Target_TCP);
        targetFrameHelper.setRotationFromMatrix(T_Target_TCP);
        targetFrameHelper.visible = true;
        fkSuccess = true;
    } else {
        console.error("FK failed for current slider angles.");
        targetFrameHelper.visible = false; // Hide target if FK fails
    }

    // 2. Inverse Kinematics for the Target Pose
    let ikSolutions = [];
    let ikTime = 0;
    if (fkSuccess && T_Target_TCP) {
        const startTime = performance.now();
        try {
             ikSolutions = inverseKinematics(T_Target_TCP);
        } catch (e) {
            console.error("Error during Inverse Kinematics:", e);
            ikSolutions = []; // Ensure it's an array on error
        }
        const endTime = performance.now();
        ikTime = endTime - startTime;

        // Draw IK solutions (blue, transparent)
        if (ikSolutions && ikSolutions.length > 0) {
            for (const solution of ikSolutions) {
                const ikAngles = solution[0];
                // Avoid re-drawing the exact FK configuration if it's found by IK
                const isFkConfig = ikAngles.every((angle, i) => Math.abs(normalizeAngle(angle - sliderJointAngles[i])) < tol_compare);

                if (!isFkConfig) {
                    const ikFkResult = forwardKinematics(ikAngles);
                    if (ikFkResult) {
                        drawRobot(ikFkResult.points, ikFkResult.T0_TCP, 'ik');
                    }
                }
            }
        }
    } else {
        // console.log("Skipping IK calculation because FK failed.");
    }


    // 3. Update Info Display
    updateInfoDisplay(sliderJointAngles, T_Target_TCP, ikSolutions, ikTime);

}

/**
 * Updates the text display with current angles, FK pose, and IK info.
 * @param {number[]} currentAngles The current slider joint angles.
 * @param {THREE.Matrix4 | null} tcpPose The calculated TCP pose from FK.
 * @param {Array} ikSolutions Array of IK solution angles.
 * @param {number} ikTime Time taken for IK calculation in ms.
 */
function updateInfoDisplay(currentAngles, tcpPose, ikSolutions, ikTime) {
    let info = `Current Angles (deg): [${currentAngles.map(a => (a * 180 / Math.PI).toFixed(1)).join(', ')}]\n`;

    if (tcpPose) {
        const pos = new THREE.Vector3().setFromMatrixPosition(tcpPose);
        const quat = new THREE.Quaternion().setFromRotationMatrix(tcpPose);
        info += `FK TCP Position (m): [${pos.x.toFixed(3)}, ${pos.y.toFixed(3)}, ${pos.z.toFixed(3)}]\n`;
        info += `FK TCP Orientation (Quat): [${quat.x.toFixed(3)}, ${quat.y.toFixed(3)}, ${quat.z.toFixed(3)}, ${quat.w.toFixed(3)}]\n`;
        info += `-------\n`;
        info += `IK Solutions Found: ${ikSolutions.length}\n`;
        info += `IK Calculation Time: ${ikTime.toFixed(1)} ms`;
        // Optional: List IK solution angles
        // if (ikSolutions.length > 0) {
        //     info += "\nIK Angles (deg):";
        //     ikSolutions.forEach((sol, idx) => {
        //         info += `\n  Sol ${idx + 1}: [${sol[0].map(a => (a * 180 / Math.PI).toFixed(1)).join(', ')}]`;
        //     });
        // }
    } else {
        info += `FK Calculation Failed.\n`;
        info += `-------\n`;
        info += `IK Not Performed.`;
    }

    infoDisplay.textContent = info;
}


// --- Initialization ---
document.addEventListener('DOMContentLoaded', () => {
    try {
        initThreeJS();
        createSliders();
        updateVisualization(); // Initial draw
        console.log("Interactive Kinematics Initialized.");
    } catch (error) {
        console.error("Initialization failed:", error);
        infoDisplay.textContent = `Error initializing visualization: ${error.message}`;
        loadingIndicator.textContent = "Error";
        loadingIndicator.style.display = 'flex'; // Keep showing error
    }
});
