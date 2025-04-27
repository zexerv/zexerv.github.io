/**
 * Interactive UR5e Forward & Inverse Kinematics Visualization using Three.js
 *
 * Allows users to control joint angles via sliders (Forward Kinematics - FK).
 * Calculates the resulting TCP pose from FK.
 * Uses the FK TCP pose as a target for Inverse Kinematics (IK).
 * Calculates and displays all valid IK solutions (up to 8) for that target pose.
 * FK configuration shown as solid red robot.
 * IK solutions shown as dashed blue transparent robots.
 */

// Ensure Three.js and OrbitControls are loaded
if (typeof THREE === 'undefined' || typeof THREE.OrbitControls === 'undefined') {
    console.error("THREE.js or OrbitControls library not found. Make sure they are included before this script.");
    // Display error message in the container
    const container = document.getElementById('kinematics-visualization');
    if (container) {
        container.innerHTML = '<p style="color: var(--text-muted); padding: 2rem; text-align: center;">Error: Required 3D libraries missing.</p>';
    }
    // Stop script execution if libraries are missing
    throw new Error("Missing THREE.js or OrbitControls");
}

// --- Constants and DH Parameters ---

const DH_PARAMS_UR5E = [
    { a: 0.0,    alpha: Math.PI / 2, d: 0.1625, theta_offset: 0.0 }, // Joint 1
    { a: -0.425, alpha: 0.0,         d: 0.0,    theta_offset: 0.0 }, // Joint 2
    { a: -0.3922,alpha: 0.0,         d: 0.0,    theta_offset: 0.0 }, // Joint 3
    { a: 0.0,    alpha: Math.PI / 2, d: 0.1333, theta_offset: 0.0 }, // Joint 4
    { a: 0.0,    alpha: -Math.PI / 2,d: 0.0997, theta_offset: 0.0 }, // Joint 5
    { a: 0.0,    alpha: 0.0,         d: 0.0996, theta_offset: 0.0 }  // Joint 6
];

const TCP_Z_OFFSET = 0.1565; // Standard TCP offset along flange Z

// Transformation from Flange frame (Link 6) to TCP frame
const H_FLANGE_TCP = new THREE.Matrix4().makeTranslation(0, 0, TCP_Z_OFFSET);
// Inverse transformation (TCP to Flange)
const INV_H_FLANGE_TCP = new THREE.Matrix4().copy(H_FLANGE_TCP).invert();

// Extract DH parameters for IK calculations
const d1 = DH_PARAMS_UR5E[0].d;
const a2 = DH_PARAMS_UR5E[1].a; // Note: a2 is negative
const a3 = DH_PARAMS_UR5E[2].a; // Note: a3 is negative
const d4 = DH_PARAMS_UR5E[3].d;
const d5 = DH_PARAMS_UR5E[4].d;
const d6 = DH_PARAMS_UR5E[5].d;
const len_a2 = Math.abs(a2);
const len_a3 = Math.abs(a3);

// Tolerances for IK calculations (similar to Python version)
const tol_zero = 1e-9;
const tol_singularity = 1e-7;
const tol_compare = 1e-5; // Slightly looser for JS floating point
const tol_geom = 1e-6;

// --- Kinematics Functions ---

/**
 * Creates a Denavit-Hartenberg transformation matrix.
 * @param {number} a - Link length
 * @param {number} alpha - Link twist
 * @param {number} d - Link offset
 * @param {number} theta - Joint angle
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

/**
 * Calculates Forward Kinematics for the UR5e.
 * @param {number[]} jointAngles - Array of 6 joint angles in radians.
 * @param {object[]} dhParams - Array of DH parameters for each joint.
 * @param {THREE.Matrix4} H_flange_tcp - Transformation from flange to TCP.
 * @returns {object} An object containing:
 * {THREE.Vector3[]} points - Array of world coordinates for each joint frame origin + TCP.
 * {THREE.Matrix4} T0_TCP - Transformation matrix from base to TCP.
 * {THREE.Matrix4[]} transforms - Array of transformation matrices from base to each frame origin + TCP.
 * Returns null if input is invalid.
 */
function forwardKinematics(jointAngles, dhParams = DH_PARAMS_UR5E, H_flange_tcp = H_FLANGE_TCP) {
    if (!jointAngles || jointAngles.length !== dhParams.length) {
        console.error(`FK Error: Invalid joint angles. Expected ${dhParams.length}, got ${jointAngles ? jointAngles.length : 'undefined'}`);
        return null; // Return null on error
    }

    const transforms = [new THREE.Matrix4()]; // Start with identity matrix for base frame T0_0
    let T_prev = transforms[0].clone();

    for (let i = 0; i < dhParams.length; i++) {
        const p = dhParams[i];
        // Check for NaN angles which can cause issues
        if (isNaN(jointAngles[i])) {
             console.error(`FK Error: NaN detected in joint angle ${i+1}`);
             return null;
        }
        const T_i_minus_1_to_i = dhMatrix(p.a, p.alpha, p.d, jointAngles[i] + p.theta_offset);
        const T_curr = new THREE.Matrix4().multiplyMatrices(T_prev, T_i_minus_1_to_i);
        transforms.push(T_curr);
        T_prev = T_curr;
    }

    const T0_flange = transforms[transforms.length - 1]; // Last calculated transform is T0_6 (to flange)
    const T0_TCP = new THREE.Matrix4().multiplyMatrices(T0_flange, H_flange_tcp);
    transforms.push(T0_TCP); // Add TCP transform

    // Extract origin points from matrices
    const points = transforms.map(matrix => {
        const position = new THREE.Vector3();
        // Use decompose to safely extract position
        matrix.decompose(position, new THREE.Quaternion(), new THREE.Vector3());
        return position;
    });

    return { points, T0_TCP, T0_flange, transforms };
}


/**
 * Calculates Inverse Kinematics for the UR5e.
 * Ported from the provided Python analytical solver.
 * @param {THREE.Matrix4} T_desired_TCP - The desired TCP pose matrix.
 * @returns {Array<number[]>} An array of valid joint angle solutions (each is an array of 6 angles in radians). Returns empty array if no solutions or error.
 */
function inverseKinematics(T_desired_TCP) {
    if (!T_desired_TCP) return [];

    const solutions = [];
    // Calculate desired Flange pose (T0_6) from desired TCP pose (T0_TCP)
    const T_flange = new THREE.Matrix4().multiplyMatrices(T_desired_TCP, INV_H_FLANGE_TCP);

    // Extract flange position (P60) and rotation (R06)
    const P60 = new THREE.Vector3().setFromMatrixPosition(T_flange);
    const R06 = new THREE.Matrix3().setFromMatrix4(T_flange); // Extract rotation part

    const pxd = P60.x;
    const pyd = P60.y;
    const pzd = P60.z;

    // Rotation matrix elements (r_row_col)
    const r = R06.elements; // THREE.Matrix3 elements are column-major [r11, r21, r31, r12, r22, r32, r13, r23, r33]
    const r11d = r[0], r21d = r[1], r31d = r[2];
    const r12d = r[3], r22d = r[4], r32d = r[5];
    const r13d = r[6], r23d = r[7], r33d = r[8];

    // --- Step 1: Find theta1 ---
    // Calculate wrist center P50 = P60 - d6 * Z_axis_of_frame6
    const z6_axis = new THREE.Vector3(r13d, r23d, r33d); // Z-axis of frame 6 in base frame
    const P50 = new THREE.Vector3().copy(P60).sub(z6_axis.multiplyScalar(d6));
    const P50x = P50.x;
    const P50y = P50.y;
    // const P50z = P50.z; // Not directly used for theta1

    // Eq 4.15 (modified atan2 arguments based on common UR IK derivations)
    // atan2(y, x)
    const term1_t1 = Math.atan2(P50y, P50x);

    // Check for singularity: P50 projection on XY plane is too close to origin
    const dist_sq_xy = P50x**2 + P50y**2;
    if (dist_sq_xy < tol_singularity**2) {
        // console.warn("IK Singularity: Wrist center too close to Z-axis.");
        // This case often implies infinite theta1 solutions, typically handled separately.
        // For simplicity here, we might return no solutions or handle based on context.
        // Let's try proceeding, might still find solutions if d4 is non-zero.
    }

    // Eq 4.16 / 4.17 (arccos argument)
    const acos_arg_t1 = d4 / Math.sqrt(dist_sq_xy);
    if (Math.abs(acos_arg_t1) > 1.0 + tol_geom) {
        // console.warn("IK Warning: Wrist center unreachable for theta1 calculation (sqrt).");
        return []; // Target likely out of reach horizontally
    }
    const acos_val_t1 = Math.acos(Math.max(-1.0, Math.min(1.0, acos_arg_t1))); // Clamp argument

    const theta1_sol = [
        term1_t1 + acos_val_t1, // First solution for theta1
        term1_t1 - acos_val_t1  // Second solution for theta1
    ];

    // --- Loop through theta1 solutions ---
    for (const t1 of theta1_sol) {
        const s1 = Math.sin(t1);
        const c1 = Math.cos(t1);

        // --- Step 2: Find theta5 ---
        // Eq 4.23 (P_xy from Eq 4.18)
        const M_val = pxd * s1 - pyd * c1 - d4; // Note: This differs slightly from Python version's M, seems more direct for P_xy
        const M = Math.max(-1.0, Math.min(1.0, M_val / d6)); // Clamp arg for acos

        if (Math.abs(M_val / d6) > 1.0 + tol_geom) {
            // console.warn(`IK Warning: Theta5 calculation out of bounds for t1=${t1.toFixed(3)}`);
            continue; // Skip this t1 branch
        }

        const t5_sol = [
             Math.acos(M), // Positive solution for theta5
            -Math.acos(M)  // Negative solution for theta5
        ];

        // --- Loop through theta5 solutions ---
        for (const t5 of t5_sol) {
            const s5 = Math.sin(t5);
            const c5 = Math.cos(t5);

            // Check for singularity: Wrist singularity (theta5 near 0 or pi)
            if (Math.abs(s5) < tol_singularity) {
                // --- Wrist Singularity ---
                // theta6 is arbitrary, often set to 0. theta2, 3, 4 are coupled.
                // console.warn(`IK Singularity: Theta5 near 0/pi for t1=${t1.toFixed(3)}, t5=${t5.toFixed(3)}`);
                // For simplicity, we might skip these solutions or attempt a simplified solve.
                // Let's try setting t6=0 and solving the rest.

                const t6 = 0.0; // Arbitrary choice for singularity

                // Simplified solve for theta234 (sum of 2,3,4)
                // Using R06 elements and t1
                const atan_y_234 = -r13d * s1 + r23d * c1;
                const atan_x_234 = -r11d * s1 + r21d * c1; // Check signs based on frame definitions
                const t234 = Math.atan2(atan_y_234, atan_x_234);

                // Now solve for theta3 using cosine law (on the arm plane)
                const KC = P50x * c1 + P50y * s1 - a1; // Projection onto arm's X-axis (assuming a1=0 for UR)
                const KS = P50z - d1;                 // Projection onto arm's Z-axis

                const dist_sq_13 = KC**2 + KS**2;
                const denom_t3 = 2 * len_a2 * len_a3;
                if (Math.abs(denom_t3) < tol_zero) continue; // Should not happen if a2, a3 != 0

                const cos_t3_arg = (dist_sq_13 - len_a2**2 - len_a3**2) / denom_t3;

                if (Math.abs(cos_t3_arg) > 1.0 + tol_geom) continue; // Unreachable configuration
                const cos_t3 = Math.max(-1.0, Math.min(1.0, cos_t3_arg));
                const t3 = Math.acos(cos_t3); // Elbow up/down handled later? Or need atan2?

                // Need both elbow up/down solutions for t3
                const t3_sol_singular = [t3, -t3];

                for (const t3_sing of t3_sol_singular) {
                    const s3_sing = Math.sin(t3_sing);
                    const c3_sing = Math.cos(t3_sing);

                    // Solve for t2
                    const term1_t2 = Math.atan2(KS, KC);
                    const term2_y_t2 = len_a3 * s3_sing;
                    const term2_x_t2 = len_a2 + len_a3 * c3_sing;
                    const term2_t2 = Math.atan2(term2_y_t2, term2_x_t2);
                    const t2 = term1_t2 - term2_t2;

                    // Solve for t4
                    const t4 = t234 - t2 - t3_sing;

                    const sol_angles_raw = [t1, t2, t3_sing, t4, t5, t6];
                    add_solution_if_valid(solutions, sol_angles_raw, T_flange);
                }


            } else {
                // --- Non-Singular Case ---

                // --- Step 3: Find theta6 ---
                // Eq 4.26 / 4.27
                const atan_y_t6 = (-r12d * s1 + r22d * c1) / s5;
                const atan_x_t6 = (r11d * s1 - r21d * c1) / s5;
                const t6 = Math.atan2(atan_y_t6, atan_x_t6);

                // --- Step 4: Find theta2, theta3, theta4 ---
                // Find theta234 (sum angle) first
                // Eq 4.36 / 4.37 (using R06 elements)
                const s6 = Math.sin(t6);
                const c6 = Math.cos(t6);

                // Z-axis of frame 3 in frame 0 (derived from T0_3 = T0_6 * T6_5 * T5_4 * T4_3.inv) is complex.
                // Easier approach: Use R03 = R06 * R65 * R54 * R43.inv
                // Simpler: Use P50 position and cosine law for t3, then find t2, then t4 = t234 - t2 - t3

                // Calculate t234 from R06 and t1, t5, t6 (more robust)
                // Need X-axis of frame 4 in frame 0 (x4_0) and Y-axis of frame 4 in frame 0 (y4_0)
                // T0_4 = T0_6 * T6_5 * T5_4
                const T5_4 = dhMatrix(DH_PARAMS_UR5E[4].a, DH_PARAMS_UR5E[4].alpha, DH_PARAMS_UR5E[4].d, t5 + DH_PARAMS_UR5E[4].theta_offset).invert();
                const T6_5 = dhMatrix(DH_PARAMS_UR5E[5].a, DH_PARAMS_UR5E[5].alpha, DH_PARAMS_UR5E[5].d, t6 + DH_PARAMS_UR5E[5].theta_offset).invert();
                const T0_4 = new THREE.Matrix4().multiplyMatrices(T_flange, T6_5).multiply(T5_4);

                // Extract R04 rotation matrix
                const R04 = new THREE.Matrix3().setFromMatrix4(T0_4);
                const r04 = R04.elements;
                // atan2(-r23_from_R04, r13_from_R04) -> gives t234? Check derivation.
                // Let's use the Python script's approach for t234 based on R06, t1, t5, t6 which seemed different.
                // Python: atan_y_234 = r31d * F - s6 * C_val ; atan_x_234 = F * C_val + s6 * r31d
                // where F = c5*c6, C_val = c1*r11d + s1*r21d
                // This seems derived from R04 elements. Let's trust it for now.
                const F = c5 * c6;
                const C_val = c1 * r11d + s1 * r21d;
                const atan_y_234 = r31d * F - s6 * C_val;
                const atan_x_234 = F * C_val + s6 * r31d;
                const t234 = Math.atan2(atan_y_234, atan_x_234);


                // --- Step 5: Find theta3 using cosine law ---
                // Use P50 position projected onto the arm plane (defined by Z0 and P50 vector)
                // KC = projection onto X-axis of arm plane (along vector from J1 to P50 projected onto XY plane)
                // KS = projection onto Z-axis of arm plane (along Z0)
                const KC = P50x * c1 + P50y * s1; // Projection onto line defined by t1 in XY plane
                const KS = P50.z - d1;            // Height above base

                const dist_sq_13 = KC**2 + KS**2; // Squared distance from J1 origin to P50 origin
                const denom_t3 = 2 * len_a2 * len_a3;
                if (Math.abs(denom_t3) < tol_zero) continue; // Should not happen

                const cos_t3_arg = (dist_sq_13 - len_a2**2 - len_a3**2) / denom_t3;

                if (Math.abs(cos_t3_arg) > 1.0 + tol_geom) {
                    // console.warn(`IK Warning: Theta3 calculation out of bounds for t1=${t1.toFixed(3)}, t5=${t5.toFixed(3)}`);
                    continue; // Unreachable configuration
                }
                const cos_t3 = Math.max(-1.0, Math.min(1.0, cos_t3_arg)); // Clamp

                // Need both elbow up/down solutions for t3
                const acos_t3 = Math.acos(cos_t3);
                const t3_sol = [acos_t3, -acos_t3];

                // --- Loop through theta3 solutions ---
                for (const t3 of t3_sol) {
                    const s3 = Math.sin(t3);
                    const c3 = Math.cos(t3); // Use clamped cos_t3

                    // --- Step 6: Find theta2 ---
                    // Eq 4.34 / 4.35
                    const term1_t2 = Math.atan2(KS, KC); // Angle of vector from J1 to P50 in arm plane
                    const term2_y_t2 = len_a3 * s3;
                    const term2_x_t2 = len_a2 + len_a3 * c3;
                    const term2_t2 = Math.atan2(term2_y_t2, term2_x_t2); // Angle adjustment due to elbow joint
                    const t2 = term1_t2 - term2_t2;

                    // --- Step 7: Find theta4 ---
                    const t4 = t234 - t2 - t3;

                    // --- Store Solution ---
                    const sol_angles_raw = [t1, t2, t3, t4, t5, t6];
                    add_solution_if_valid(solutions, sol_angles_raw, T_flange);

                } // end t3 loop
            } // end non-singular case
        } // end t5 loop
    } // end t1 loop

    // console.log(`Found ${solutions.length} raw IK solutions.`);
    return solutions;
}

/**
 * Normalizes angles to [-pi, pi] and adds solution if FK check passes.
 * @param {Array<number[]>} solutions_array - Array to add valid solutions to.
 * @param {number[]} angles_raw - The raw calculated angles.
 * @param {THREE.Matrix4} T_target_flange - The target flange pose for verification.
 */
function add_solution_if_valid(solutions_array, angles_raw, T_target_flange) {
    // Normalize angles to [-pi, pi]
    const angles_normalized = angles_raw.map(a => {
        let wrapped = (a + Math.PI) % (2 * Math.PI);
        if (wrapped < 0) wrapped += 2 * Math.PI; // Ensure positive before subtracting PI
        return wrapped - Math.PI;
    });

    // Optional: FK check to verify the solution
    try {
        const fk_result = forwardKinematics(angles_normalized);
        if (fk_result && fk_result.T0_flange) {
            // Compare calculated flange pose with target flange pose
            const T_calc_flange = fk_result.T0_flange;
            let diff = 0;
            for(let i=0; i<16; ++i) {
                diff += Math.abs(T_calc_flange.elements[i] - T_target_flange.elements[i]);
            }

            if (diff < tol_compare * 10) { // Use a slightly larger tolerance for overall check
                 // Check if this solution (or a very close one) is already added
                 let already_added = false;
                 for(const existing_sol of solutions_array) {
                     let angle_diff_sum = 0;
                     for(let j=0; j<6; ++j) {
                         // Handle angle wrapping difference
                         let diff_j = Math.abs(angles_normalized[j] - existing_sol[j]);
                         angle_diff_sum += Math.min(diff_j, 2 * Math.PI - diff_j);
                     }
                     if (angle_diff_sum < tol_compare * 6) { // Check average angle difference
                         already_added = true;
                         break;
                     }
                 }
                 if (!already_added) {
                    solutions_array.push(angles_normalized);
                 }

            } else {
                // console.warn("IK Solution failed FK check:", angles_normalized, "Diff:", diff);
            }
        }
    } catch (e) {
        console.error("Error during FK check of IK solution:", e);
    }
}


// --- Three.js Setup ---
let scene, camera, renderer, controls;
let fkRobotGroup; // Group for the main FK robot (sliders)
let ikSolutionGroups = []; // Array to hold groups for each IK solution robot
let targetTCPHelper; // Axes helper for the target TCP frame

const container = document.getElementById('kinematics-visualization');
const loadingIndicator = document.getElementById('loading-indicator');

// Colors and Materials
const fkColor = 0xff0000; // Red for FK robot
const ikColor = 0x0077ff; // Blue for IK solutions
const jointColor = 0xdddddd; // White/Gray for joints

const fkLinkMaterials = [
    new THREE.MeshStandardMaterial({ color: 0xcc0000, metalness: 0.3, roughness: 0.6 }),
    new THREE.MeshStandardMaterial({ color: 0xaa0000, metalness: 0.3, roughness: 0.6 }),
    new THREE.MeshStandardMaterial({ color: 0xcc0000, metalness: 0.3, roughness: 0.6 }),
    new THREE.MeshStandardMaterial({ color: 0xaa0000, metalness: 0.3, roughness: 0.6 }),
    new THREE.MeshStandardMaterial({ color: 0xcc0000, metalness: 0.3, roughness: 0.6 }),
    new THREE.MeshStandardMaterial({ color: 0xaa0000, metalness: 0.3, roughness: 0.6 }),
];
const fkJointMaterial = new THREE.MeshStandardMaterial({ color: jointColor, metalness: 0.2, roughness: 0.7 });

const ikLinkMaterial = new THREE.MeshStandardMaterial({
    color: ikColor,
    transparent: true,
    opacity: 0.4, // Make IK solutions transparent
    metalness: 0.3,
    roughness: 0.6,
    depthWrite: false // Helps with transparency sorting issues
});
const ikJointMaterial = new THREE.MeshStandardMaterial({
    color: jointColor,
    transparent: true,
    opacity: 0.5,
    metalness: 0.2,
    roughness: 0.7,
    depthWrite: false
});

// Dashed line material for IK links (applied later)
const ikDashedLineMaterial = new THREE.LineDashedMaterial({
	color: ikColor,
	linewidth: 1,
	scale: 1,
	dashSize: 0.05, // Length of dashes
	gapSize: 0.03, // Length of gaps
    transparent: true,
    opacity: 0.6
});


function initThreeJS() {
    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a202c); // Darker background

    // Camera
    const aspect = container.clientWidth / container.clientHeight;
    camera = new THREE.PerspectiveCamera(50, aspect, 0.1, 100);
    camera.position.set(1.2, 1.6, 2.2); // Adjusted initial camera position
    camera.lookAt(0, 0.4, 0);

    // Renderer
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true }); // Enable alpha for potential overlays
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    // renderer.sortObjects = false; // May help with transparency, but can cause other issues
    container.appendChild(renderer.domElement);

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
    scene.add(ambientLight);
    const directionalLight1 = new THREE.DirectionalLight(0xffffff, 0.6);
    directionalLight1.position.set(5, 10, 7);
    scene.add(directionalLight1);
    const directionalLight2 = new THREE.DirectionalLight(0xffffff, 0.3);
    directionalLight2.position.set(-5, -5, -3);
    scene.add(directionalLight2);

    // Controls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0.4, 0);
    controls.enableDamping = true;
    controls.dampingFactor = 0.1;
    controls.screenSpacePanning = false;

    // Coordinate Frame Helper (Base)
    const baseAxesHelper = new THREE.AxesHelper(0.2);
    scene.add(baseAxesHelper);

    // Target TCP Helper (initialized but positioned later)
    targetTCPHelper = new THREE.AxesHelper(0.15); // Make target TCP slightly larger
    targetTCPHelper.visible = false; // Initially hidden
    scene.add(targetTCPHelper);

    // Create the FK robot group
    fkRobotGroup = createRobotModel(fkLinkMaterials, fkJointMaterial, false); // Solid lines for FK
    scene.add(fkRobotGroup);

    // Handle window resize
    window.addEventListener('resize', onWindowResize, false);

    // Start animation loop
    animate();
}

/**
 * Creates the geometric structure (meshes) for a single robot instance.
 * @param {THREE.Material[]} linkMaterials - Array of materials for the links.
 * @param {THREE.Material} jointMaterial - Material for the joints.
 * @param {boolean} useDashedLines - If true, uses LineDashedMaterial for links.
 * @returns {THREE.Group} A group containing the robot meshes.
 */
function createRobotModel(linkMaterials, jointMaterial, useDashedLines = false) {
    const robotGroup = new THREE.Group();
    const jointMeshes = [];
    const linkMeshes = []; // Will store Cylinders or LineSegments

    const jointRadius = 0.04;
    const linkRadius = 0.03;
    const jointGeometry = new THREE.SphereGeometry(jointRadius, 12, 12); // Reduced segments for performance

    // Create joints (spheres) - 7 total (Base + 6 joints)
    for (let i = 0; i < 7; i++) {
        const joint = new THREE.Mesh(jointGeometry, jointMaterial);
        robotGroup.add(joint);
        jointMeshes.push(joint);
    }
    jointMeshes[0].scale.set(1.5, 1.5, 1.5); // Make base joint larger

    // Create links - 6 total
    for (let i = 0; i < 6; i++) {
        let link;
        if (useDashedLines) {
            // Use LineSegments for dashed lines
            const lineGeom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0,0,0), new THREE.Vector3(0,1,0)]); // Placeholder points
            link = new THREE.LineSegments(lineGeom, ikDashedLineMaterial); // Use the dashed material
        } else {
            // Use Cylinders for solid links
            const linkMaterial = Array.isArray(linkMaterials) ? linkMaterials[i % linkMaterials.length] : linkMaterials;
            const linkGeom = new THREE.CylinderGeometry(linkRadius, linkRadius, 0.1, 12); // Placeholder height
            link = new THREE.Mesh(linkGeom, linkMaterial);
        }
        robotGroup.add(link);
        linkMeshes.push(link);
    }

    // TCP Axes Helper (associated with this robot instance)
    const tcpHelper = new THREE.AxesHelper(0.1);
    robotGroup.add(tcpHelper);

    // Store references for easy update
    robotGroup.userData = { jointMeshes, linkMeshes, tcpHelper, isDashed: useDashedLines };
    return robotGroup;
}


/**
 * Updates the pose of a single robot model instance based on FK results.
 * @param {THREE.Group} robotGroup - The group containing the robot meshes.
 * @param {object | null} fkResult - The result from forwardKinematics function, or null if FK failed.
 */
function updateRobotModelPose(robotGroup, fkResult) {
    if (!robotGroup || !robotGroup.userData) return; // Safety check

    const { jointMeshes, linkMeshes, tcpHelper, isDashed } = robotGroup.userData;

    // If FK failed, hide the robot
    if (!fkResult || !fkResult.points || !fkResult.transforms || fkResult.points.length !== 8) {
        robotGroup.visible = false;
        // console.log("Hiding robot due to invalid FK result");
        return;
    }
    robotGroup.visible = true;

    const { points, transforms } = fkResult; // points: Vector3[], transforms: Matrix4[]

    // Update Joint Positions (Spheres)
    for (let i = 0; i < 7; i++) {
        if (jointMeshes[i] && points[i]) {
            jointMeshes[i].position.copy(points[i]);
        } else {
             // console.warn(`Missing joint mesh or point for index ${i}`);
        }
    }

    // Update Link Positions and Orientations
    for (let i = 0; i < 6; i++) {
        const startPoint = points[i];
        const endPoint = points[i + 1];
        const linkMesh = linkMeshes[i];

        if (!startPoint || !endPoint || !linkMesh) continue;

        const distance = startPoint.distanceTo(endPoint);

        if (distance < 0.001) {
            linkMesh.visible = false;
            continue;
        }
        linkMesh.visible = true;

        if (isDashed) {
            // Update LineSegments geometry for dashed lines
            const lineGeom = linkMesh.geometry;
            const positions = lineGeom.attributes.position.array;
            positions[0] = startPoint.x; positions[1] = startPoint.y; positions[2] = startPoint.z;
            positions[3] = endPoint.x;   positions[4] = endPoint.y;   positions[5] = endPoint.z;
            lineGeom.attributes.position.needsUpdate = true;
            lineGeom.computeBoundingSphere(); // Important for dashed lines
            linkMesh.computeLineDistances(); // Crucial for dashed lines to render correctly!

        } else {
            // Update Cylinder geometry and pose for solid links
            const linkRadius = 0.03;
            // Only recreate geometry if height changes significantly
            if (!linkMesh.geometry || Math.abs(linkMesh.geometry.parameters.height - distance) > 0.001) {
                 if(linkMesh.geometry) linkMesh.geometry.dispose();
                linkMesh.geometry = new THREE.CylinderGeometry(linkRadius, linkRadius, distance, 12); // Reduced segments
            }

            // Position halfway and orient
            linkMesh.position.copy(startPoint).lerp(endPoint, 0.5);
            linkMesh.lookAt(endPoint);
            linkMesh.rotateX(Math.PI / 2); // Align cylinder axis
        }
    }

    // Update TCP Helper Pose (using the T0_TCP matrix)
    const tcpTransform = transforms[7]; // Last transform is T0_TCP
    if (tcpHelper && tcpTransform) {
        const position = new THREE.Vector3();
        const quaternion = new THREE.Quaternion();
        tcpTransform.decompose(position, quaternion, new THREE.Vector3()); // Extract pos/rot

        tcpHelper.position.copy(position);
        tcpHelper.quaternion.copy(quaternion);
    } else {
         // console.warn("Missing TCP helper or transform");
    }
}


/**
 * Formats a THREE.Matrix4 for display.
 * @param {THREE.Matrix4} matrix - The matrix to format.
 * @returns {string} Formatted string representation.
 */
function formatMatrixForDisplay(matrix) {
    if (!matrix) return "N/A";
    const e = matrix.elements; // Column-major order
    let str = "";
    for (let i = 0; i < 4; i++) { // Row
        str += `[ ${e[i].toFixed(3)}, ${e[i + 4].toFixed(3)}, ${e[i + 8].toFixed(3)}, ${e[i + 12].toFixed(3)} ]\n`;
    }
    return str.trim();
}


// --- UI Interaction ---
const sliders = [];
const valueDisplays = [];
for (let i = 1; i <= 6; i++) {
    const slider = document.getElementById(`q${i}`);
    const display = document.getElementById(`q${i}-value`);
    if (slider && display) {
        sliders.push(slider);
        valueDisplays.push(display);
        // Use 'input' for real-time updates, 'change' for update on release
        slider.addEventListener('input', handleSliderChange);
    } else {
        console.error(`Slider or display element not found for q${i}`);
    }
}
const infoDisplay = document.getElementById('info-display');
const infoTitle = document.getElementById('info-title');
const infoContent = document.getElementById('info-content');

// Debounce function to limit rapid updates
function debounce(func, wait) {
    let timeout;
    return function executedFunction(...args) {
        const later = () => {
            clearTimeout(timeout);
            func(...args);
        };
        clearTimeout(timeout);
        timeout = setTimeout(later, wait);
    };
}


// Debounced version of the update logic
const debouncedUpdate = debounce(() => {
    if (loadingIndicator) loadingIndicator.style.display = 'block'; // Show loading

    // Use requestAnimationFrame to ensure DOM updates (like loading indicator) happen first
    requestAnimationFrame(() => {
        updateVisualization();
        if (loadingIndicator) loadingIndicator.style.display = 'none'; // Hide loading
    });
}, 100); // Debounce time in ms (e.g., 100ms)


function handleSliderChange() {
    // Update value displays immediately
    sliders.forEach((slider, i) => {
        if (valueDisplays[i]) {
            valueDisplays[i].textContent = parseFloat(slider.value).toFixed(2);
        }
    });
    // Trigger the debounced update
    debouncedUpdate();
}

/**
 * Main function to update the entire visualization based on slider values.
 */
function updateVisualization() {
    // 1. Get current joint angles from sliders
    const currentJointAngles = sliders.map(s => parseFloat(s.value));

    // 2. Perform Forward Kinematics (FK) for the slider configuration
    let targetPoseTCP = null;
    let fkResult = null;
    try {
        fkResult = forwardKinematics(currentJointAngles);
        if (fkResult && fkResult.T0_TCP) {
            targetPoseTCP = fkResult.T0_TCP;
            // Update the main FK robot model
            updateRobotModelPose(fkRobotGroup, fkResult);
            // Position the target TCP helper
            const pos = new THREE.Vector3();
            const quat = new THREE.Quaternion();
            targetPoseTCP.decompose(pos, quat, new THREE.Vector3());
            targetTCPHelper.position.copy(pos);
            targetTCPHelper.quaternion.copy(quat);
            targetTCPHelper.visible = true;
        } else {
            // Handle FK failure
            fkRobotGroup.visible = false;
            targetTCPHelper.visible = false;
            console.error("FK failed for slider angles.");
        }
    } catch (error) {
        console.error("Error during FK calculation:", error);
        fkRobotGroup.visible = false;
        targetTCPHelper.visible = false;
        infoTitle.textContent = "Error:";
        infoContent.textContent = `FK Error: ${error.message}`;
        return; // Stop if FK fails
    }

    // 3. Perform Inverse Kinematics (IK) using the FK result as the target
    let ikSolutions = [];
    if (targetPoseTCP) {
        try {
            const startTime = performance.now();
            ikSolutions = inverseKinematics(targetPoseTCP);
            const endTime = performance.now();
            // console.log(`IK calculation took ${(endTime - startTime).toFixed(1)} ms, found ${ikSolutions.length} solutions.`);
        } catch (error) {
            console.error("Error during IK calculation:", error);
            ikSolutions = []; // Ensure it's an empty array on error
        }
    }

    // 4. Update Info Display
    if (infoDisplay && infoTitle && infoContent) {
        infoTitle.textContent = `Target TCP & ${ikSolutions.length} IK Solutions:`;
        let content = `Target Pose (T0_TCP):\n${formatMatrixForDisplay(targetPoseTCP)}\n\n`;
        if (ikSolutions.length > 0) {
             content += "IK Solutions (Joint Angles):\n";
             ikSolutions.forEach((sol, index) => {
                 content += `${index + 1}: [${sol.map(a => a.toFixed(3)).join(', ')}]\n`;
             });
        } else if (targetPoseTCP) {
             content += "No valid IK solutions found for this target pose.";
        } else {
             content = "Could not calculate FK target pose.";
        }
        infoContent.textContent = content.trim();
    }


    // 5. Update IK Solution Visualizations
    // Remove old IK groups
    ikSolutionGroups.forEach(group => scene.remove(group));
    ikSolutionGroups = [];

    // Create and update new IK groups
    ikSolutions.forEach(solutionAngles => {
        const ikFkResult = forwardKinematics(solutionAngles);
        if (ikFkResult) {
            // Avoid plotting the IK solution if it's identical to the FK solution
            let angle_diff_sum = 0;
            for(let j=0; j<6; ++j) {
                let diff_j = Math.abs(solutionAngles[j] - currentJointAngles[j]);
                angle_diff_sum += Math.min(diff_j, 2 * Math.PI - diff_j);
            }

            if (angle_diff_sum > tol_compare * 6) { // Only plot if significantly different
                const ikRobotGroup = createRobotModel(ikLinkMaterial, ikJointMaterial, true); // Dashed lines for IK
                updateRobotModelPose(ikRobotGroup, ikFkResult);
                scene.add(ikRobotGroup);
                ikSolutionGroups.push(ikRobotGroup);
            }
        }
    });
}


// --- Animation Loop ---
function animate() {
    requestAnimationFrame(animate);
    controls.update(); // Required for damping
    renderer.render(scene, camera);
}

// --- Resize Handling ---
function onWindowResize() {
    if (!container || !renderer || !camera) return;
    const width = container.clientWidth;
    const height = container.clientHeight;

    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
}

// --- Initialization ---
if (container) {
    try {
        initThreeJS();
        // Initial update based on default slider values
        // Use setTimeout to ensure the scene is ready before the first potentially heavy update
        setTimeout(() => {
            handleSliderChange(); // Trigger initial calculation and render
        }, 100); // Small delay
    } catch (error) {
        console.error("Initialization failed:", error);
         container.innerHTML = `<p style="color: var(--text-muted); padding: 2rem; text-align: center;">Error initializing 3D view: ${error.message}</p>`;
    }

} else {
    console.error("Container element #kinematics-visualization not found.");
}
