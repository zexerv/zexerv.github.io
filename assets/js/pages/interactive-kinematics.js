import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// --- UR5e Constants ---
const DH_PARAMS_UR5E = [
    { a: 0.0,    alpha: Math.PI / 2, d: 0.1625, theta_offset: 0.0 },
    { a: -0.425, alpha: 0.0,         d: 0.0,    theta_offset: 0.0 },
    { a: -0.3922,alpha: 0.0,         d: 0.0,    theta_offset: 0.0 },
    { a: 0.0,    alpha: Math.PI / 2, d: 0.1333, theta_offset: 0.0 },
    { a: 0.0,    alpha: -Math.PI / 2,d: 0.0997, theta_offset: 0.0 },
    { a: 0.0,    alpha: 0.0,         d: 0.0996, theta_offset: 0.0 }
];

const TCP_Z_OFFSET = 0.1565; // Example TCP offset, adjust if needed

// Transformation from Flange (Joint 6) to TCP
const H_FLANGE_TCP = new THREE.Matrix4().makeTranslation(0, 0, TCP_Z_OFFSET);
// Inverse Transformation
const INV_H_FLANGE_TCP = new THREE.Matrix4().copy(H_FLANGE_TCP).invert();

// Extract DH parameters for easier access in IK
const d1 = DH_PARAMS_UR5E[0].d;
const a2 = DH_PARAMS_UR5E[1].a; // Signed: -0.425
const a3 = DH_PARAMS_UR5E[2].a; // Signed: -0.3922
const d4 = DH_PARAMS_UR5E[3].d;
const d5 = DH_PARAMS_UR5E[4].d;
const d6 = DH_PARAMS_UR5E[5].d;
const len_a2 = Math.abs(a2); // Absolute value for specific calculations matching Python source
const len_a3 = Math.abs(a3); // Absolute value for specific calculations matching Python source


// Tolerances (from Python)
const tol_zero = 1e-9;
const tol_singularity = 1e-7;
const tol_compare = 1e-5; // Verification tolerance
const tol_geom = 1e-6;

// --- Utility Functions ---

/**
 * Clamps a value between a minimum and maximum.
 */
function clamp(value, min, max) {
    return Math.max(min, Math.min(value, max));
}

/**
 * Normalizes an angle to the range [-PI, PI].
 */
function normalizeAngle(angle) {
    angle = angle % (2 * Math.PI);
    if (angle > Math.PI) {
        angle -= 2 * Math.PI;
    } else if (angle <= -Math.PI) {
        angle += 2 * Math.PI;
    }
    return angle;
}


/**
 * Creates a Denavit-Hartenberg transformation matrix.
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
 */
function forwardKinematics(jointAngles, dhParams = DH_PARAMS_UR5E, HFlangeTcp = H_FLANGE_TCP) {
    // Input validation
    if (!Array.isArray(jointAngles) || jointAngles.length !== dhParams.length) {
        console.error(`FK Error: Invalid jointAngles input. Expected array of length ${dhParams.length}. Got:`, jointAngles);
        return null;
    }
    if (jointAngles.some(isNaN)) {
         console.error(`FK Error: jointAngles contains NaN values:`, jointAngles);
         return null;
    }

    const transforms = [new THREE.Matrix4()]; // Base frame (identity)
    const points = [new THREE.Vector3(0, 0, 0)]; // Base origin
    let T_prev = transforms[0].clone();

    try {
        for (let i = 0; i < dhParams.length; i++) {
            const p = dhParams[i];
            const currentAngle = jointAngles[i] + p.theta_offset;
            if (isNaN(currentAngle)) {
                 console.error(`FK Error: Calculated angle for joint ${i} is NaN. Input angle: ${jointAngles[i]}`);
                 return null;
            }
            const T_i_minus_1_to_i = dhMatrix(p.a, p.alpha, p.d, currentAngle);
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
    } catch (error) {
         console.error("FK Error during matrix calculations:", error);
         return null;
    }
}


/**
 * Calculates Inverse Kinematics for the UR5e.
 * VERSION 3 - Aligning t5 and t3 calculations strictly with provided Python source.
 * @param {THREE.Matrix4} T_desired_TCP The desired TCP pose matrix relative to the base.
 * @param {object[]} dhParams DH parameters array.
 * @param {THREE.Matrix4} invHFlangeTcp Inverse transformation from TCP to flange.
 * @returns {Array<[number[], number, number, number]>} Array of solutions. Each solution is [angles[6], t1_idx, t5_idx, t3_idx]. Returns empty array if no solutions found.
 */
function inverseKinematics(T_desired_TCP, dhParams = DH_PARAMS_UR5E, invHFlangeTcp = INV_H_FLANGE_TCP) {
    const solutions = [];
    const T_flange = new THREE.Matrix4().multiplyMatrices(T_desired_TCP, invHFlangeTcp);

    const P60 = new THREE.Vector3().setFromMatrixPosition(T_flange);
    const R06 = new THREE.Matrix3().setFromMatrix4(T_flange);

    const pxd = P60.x;
    const pyd = P60.y;
    const pzd = P60.z;

    const r11d = R06.elements[0]; const r12d = R06.elements[3]; const r13d = R06.elements[6];
    const r21d = R06.elements[1]; const r22d = R06.elements[4]; const r23d = R06.elements[7];
    const r31d = R06.elements[2]; const r32d = R06.elements[5]; const r33d = R06.elements[8];

    // --- Theta 1 ---
    const P50 = new THREE.Vector3(pxd - d6 * r13d, pyd - d6 * r23d, pzd - d6 * r33d);
    const P50x = P50.x; // Python 'B'
    const P50y = P50.y; // Python 'A'
    const P50z = P50.z;

    // console.log(`--- IK Start ---`);
    // console.log(`P60: [${pxd.toFixed(4)}, ${pyd.toFixed(4)}, ${pzd.toFixed(4)}]`);
    // console.log(`P50: [${P50x.toFixed(4)}, ${P50y.toFixed(4)}, ${P50z.toFixed(4)}]`);

    const dist_sq_xy = P50x * P50x + P50y * P50y;
    const sqrt_arg_t1_sq = dist_sq_xy - d4 * d4;

    if (sqrt_arg_t1_sq < -tol_geom) {
        // console.log(`IK Fail: t1 out of reach (sqrt_arg_t1_sq = ${sqrt_arg_t1_sq.toFixed(6)})`);
        return [];
    }
    const sqrt_arg_t1 = Math.max(0, sqrt_arg_t1_sq);
    const sqrt_val_t1 = Math.sqrt(sqrt_arg_t1);

    if (Math.abs(P50x) < tol_singularity && Math.abs(P50y) < tol_singularity) {
        // console.log(`IK Fail: t1 singularity (P50x, P50y near zero)`);
        return [];
    }

    const term1_t1 = Math.atan2(P50x, -P50y); // Matches Python atan2(B, -A)
    const term2_t1 = Math.atan2(sqrt_val_t1, d4); // Matches Python atan2(sqrt, d4)

    const theta1_sol = [
        normalizeAngle(term1_t1 + term2_t1),
        normalizeAngle(term1_t1 - term2_t1)
    ];
    // console.log(`t1 sols (rad): [${theta1_sol[0].toFixed(4)}, ${theta1_sol[1].toFixed(4)}]`);

    // --- Iterate Theta 1 ---
    for (let t1_idx = 0; t1_idx < theta1_sol.length; t1_idx++) {
        const t1 = theta1_sol[t1_idx];
        const s1 = Math.sin(t1);
        const c1 = Math.cos(t1);
        // console.log(`\nProcessing t1_idx=${t1_idx}, t1=${t1.toFixed(4)}`);

        // --- Theta 5 (Strictly following Python source logic) ---
        // Python M_val = s1 * r13d - c1 * r23d
        const M_val = s1 * r13d - c1 * r23d; // This is M in Python source for t5 calc
        const M = clamp(M_val, -1.0, 1.0);

        // Python D = c1 * r22d - s1 * r12d
        // Python E = s1 * r11d - c1 * r21d
        const D_py = c1 * r22d - s1 * r12d;
        const E_py = s1 * r11d - c1 * r21d;

        const sqrt_arg_ED_sq = E_py * E_py + D_py * D_py;
        // Python source check: if abs(M**2 + sqrt_arg_ED - 1.0) > tol_compare: continue
        // This seems like a verification step in Python, let's check it
        if (Math.abs(M * M + sqrt_arg_ED_sq - 1.0) > tol_compare) {
             // console.log(`IK Fail: t1_idx=${t1_idx}: M/D/E consistency check failed (M^2 + D^2 + E^2 != 1). Diff=${Math.abs(M*M + sqrt_arg_ED_sq - 1.0).toExponential(3)}`);
             continue; // Skip this t1 if orientation matrix is inconsistent
        }

        const sqrt_arg_ED = Math.max(0, sqrt_arg_ED_sq);
        const sqrt_val_ED = Math.sqrt(sqrt_arg_ED); // This is |sin(t5)| in Python's logic

        // Python source check: if abs(sqrt_val_ED) < tol_singularity and abs(M) < tol_singularity: continue
        if (sqrt_val_ED < tol_singularity && Math.abs(M) < tol_singularity) {
            // console.log(`IK Fail: t1_idx=${t1_idx}: t5 singularity (M and sqrt_val_ED near zero)`);
            continue;
        }

        // Python source: t5_sol = [math.atan2(sqrt_val_ED, M), math.atan2(-sqrt_val_ED, M)]
        const t5_sol = [
            normalizeAngle(Math.atan2(sqrt_val_ED, M)),  // atan2( |sin(t5)|, cos(t5) )
            normalizeAngle(Math.atan2(-sqrt_val_ED, M)) // atan2( -|sin(t5)|, cos(t5) )
        ];
        // console.log(` t5 sols (rad): [${t5_sol[0].toFixed(4)}, ${t5_sol[1].toFixed(4)}] (M=${M.toFixed(4)}, sqrt_ED=${sqrt_val_ED.toFixed(4)})`);

        // --- Iterate Theta 5 ---
        for (let t5_idx = 0; t5_idx < t5_sol.length; t5_idx++) {
            const t5 = t5_sol[t5_idx];
            const s5 = Math.sin(t5); // Need sine for t6 logic
            const c5 = Math.cos(t5); // Need cosine for KC/KS
            // console.log(`  Processing t5_idx=${t5_idx}, t5=${t5.toFixed(4)}`);

            // --- Theta 6 ---
            let t6 = 0.0;
            const is_singular_s5 = Math.abs(s5) < tol_singularity;

            if (is_singular_s5) {
                // console.log(`   t6 singularity (s5=${s5.toExponential(3)}), setting t6=0`);
                t6 = 0.0;
            } else {
                // Use D_py, E_py calculated earlier
                if (Math.abs(D_py) < tol_singularity && Math.abs(E_py) < tol_singularity) {
                    // console.log(`   IK Fail: t5_idx=${t5_idx}: D_py and E_py near zero in t6 calculation.`);
                    continue;
                }
                // Python logic: if s5 > 0: t6 = atan2(D, E) else: t6 = atan2(-D, -E)
                if (s5 > 0) {
                    t6 = normalizeAngle(Math.atan2(D_py, E_py));
                } else {
                    t6 = normalizeAngle(Math.atan2(-D_py, -E_py));
                }
            }
            // console.log(`   t6 (rad): ${t6.toFixed(4)} (D_py=${D_py.toFixed(4)}, E_py=${E_py.toFixed(4)}, s5=${s5.toFixed(4)})`);

            // --- Theta 3 (Strictly following Python source logic) ---
            const s6 = Math.sin(t6);
            const c6 = Math.cos(t6);

            // t234 calculation seems standard
            const atan_y_234 = -s5 * (r13d * c1 + r23d * s1); // = -s5 * (Standard M from other derivations)
            const atan_x_234 = r33d;
            if (Math.abs(atan_y_234) < tol_singularity && Math.abs(atan_x_234) < tol_singularity) {
                // console.log(`   IK Fail: t5_idx=${t5_idx}: atan_y/x_234 near zero for t234 calc.`);
                continue;
            }
            const t234 = Math.atan2(atan_y_234, atan_x_234);
            const s234 = Math.sin(t234);
            const c234 = Math.cos(t234);
            // console.log(`   t234 (rad): ${t234.toFixed(4)}`);

            // KC/KS using Python source formulas
            const KC = c1 * pxd + s1 * pyd - s234 * d5 + c234 * s5 * d6;
            const KS = pzd - d1 + c234 * d5 + s234 * s5 * d6;
            // console.log(`   KC=${KC.toFixed(4)}, KS=${KS.toFixed(4)}`);

            const dist_sq_13 = KC * KC + KS * KS;
            // Denominator and Numerator using len_a2, len_a3 as per Python source
            const denom_t3 = 2 * len_a2 * len_a3;
            if (Math.abs(denom_t3) < tol_zero) {
                // console.log(`   IK Fail: t5_idx=${t5_idx}: Denominator for t3 is zero.`);
                continue;
            }
            const cos_t3_arg = (dist_sq_13 - len_a2 * len_a2 - len_a3 * len_a3) / denom_t3;
            // console.log(`   cos_t3_arg: ${cos_t3_arg.toFixed(6)} (dist_sq=${dist_sq_13.toFixed(4)}, len_a2^2=${(len_a2*len_a2).toFixed(4)}, len_a3^2=${(len_a3*len_a3).toFixed(4)}, denom=${denom_t3.toFixed(4)})`);


            if (Math.abs(cos_t3_arg) > 1.0 + tol_geom) {
                // console.log(`   IK Fail: t5_idx=${t5_idx}: Elbow out of reach |cos(t3)| > 1. Arg: ${cos_t3_arg.toFixed(6)}`);
                continue;
            }
            const cos_t3 = clamp(cos_t3_arg, -1.0, 1.0);

            const sqrt_arg_t3_sq = 1.0 - cos_t3 * cos_t3;
             if (sqrt_arg_t3_sq < -tol_geom) {
                 // console.log(`   IK Fail: t5_idx=${t5_idx}: Invalid state for theta3 calculation (sqrt_arg < 0). cos_t3=${cos_t3.toFixed(6)}`);
                 continue;
             }
            const sqrt_arg_t3 = Math.max(0, sqrt_arg_t3_sq);
            const sqrt_val_t3 = Math.sqrt(sqrt_arg_t3); // |sin(t3)|

            const t3_sol = [
                normalizeAngle(Math.atan2(sqrt_val_t3, cos_t3)),
                normalizeAngle(Math.atan2(-sqrt_val_t3, cos_t3))
            ];
            // console.log(`    t3 sols (rad): [${t3_sol[0].toFixed(4)}, ${t3_sol[1].toFixed(4)}]`);

            // --- Iterate Theta 3 ---
            for (let t3_idx = 0; t3_idx < t3_sol.length; t3_idx++) {
                const t3 = t3_sol[t3_idx];
                const s3 = Math.sin(t3);
                const c3 = cos_t3; // Use the calculated cosine
                // console.log(`     Processing t3_idx=${t3_idx}, t3=${t3.toFixed(4)}`);

                // --- Theta 2 ---
                const term1_t2_y = KS;
                const term1_t2_x = KC;
                 if (Math.abs(term1_t2_y) < tol_singularity && Math.abs(term1_t2_x) < tol_singularity) {
                    // console.log(`     IK Fail: t3_idx=${t3_idx}: KS and KC near zero in t2 calculation.`);
                    continue;
                 }
                const term1_t2 = Math.atan2(term1_t2_y, term1_t2_x);

                // Use SIGNED a2, a3 as per Python source
                const term2_t2_y = a3 * s3;
                const term2_t2_x = a2 + a3 * c3;
                 if (Math.abs(term2_t2_y) < tol_singularity && Math.abs(term2_t2_x) < tol_singularity) {
                    // console.log(`     IK Fail: t3_idx=${t3_idx}: term2 y/x near zero in t2 calculation.`);
                    continue;
                 }
                const term2_t2 = Math.atan2(term2_t2_y, term2_t2_x);

                const t2 = normalizeAngle(term1_t2 - term2_t2);
                // console.log(`     t2 (rad): ${t2.toFixed(4)} (term1=${term1_t2.toFixed(4)}, term2=${term2_t2.toFixed(4)})`);

                // --- Theta 4 ---
                const t4 = normalizeAngle(t234 - t2 - t3);
                // console.log(`     t4 (rad): ${t4.toFixed(4)}`);

                // --- Store Solution ---
                const sol_angles_raw = [t1, t2, t3, t4, t5, t6];
                const sol_angles_normalized = sol_angles_raw.map(normalizeAngle);
                // console.log(`     Solution Candidate (rad): [${sol_angles_normalized.map(a => a.toFixed(3)).join(', ')}]`);

                // --- Verification ---
                const fkCheck = forwardKinematics(sol_angles_normalized, dhParams, H_FLANGE_TCP);
                if (fkCheck) {
                    const T_check_TCP = fkCheck.T0_TCP;
                    let diff = 0;
                    for(let k=0; k<16; k++) {
                        diff += Math.abs(T_check_TCP.elements[k] - T_desired_TCP.elements[k]);
                    }
                    const avgDiff = diff / 16;

                    if (avgDiff < tol_compare) {
                         // console.log(`     SUCCESS: Solution verified. Avg Diff: ${avgDiff.toExponential(3)}`);
                         solutions.push([sol_angles_normalized, t1_idx, t5_idx, t3_idx]);
                    } else {
                         // console.warn(`     VERIFY FAIL: FK mismatch. Avg Diff: ${avgDiff.toExponential(3)} > Tol: ${tol_compare.toExponential(3)}`);
                         // console.log("      Target Matrix:", T_desired_TCP.elements.map(n=>n.toFixed(3)));
                         // console.log("      Actual Matrix:", T_check_TCP.elements.map(n=>n.toFixed(3)));
                         // console.log("      Failed Angles (rad):", sol_angles_normalized.map(a => a.toFixed(3)));
                    }
                } else {
                     console.error(`     VERIFY ERROR: FK failed for solution candidate [${sol_angles_normalized.map(a=>a.toFixed(3))}].`);
                }

            } // end t3 loop
        } // end t5 loop
    } // end t1 loop

    // Remove duplicate solutions
    const uniqueSolutions = [];
    const seenSolutions = new Set();
    for (const sol of solutions) {
        const key = sol[0].map(a => a.toFixed(5)).join(',');
        if (!seenSolutions.has(key)) {
            uniqueSolutions.push(sol);
            seenSolutions.add(key);
        } else {
            // console.log("IK Debug: Duplicate solution removed:", key);
        }
    }
    // console.log(`--- IK End: Found ${uniqueSolutions.length} unique solutions ---`);
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

// Create materials once
const jointMaterial = new THREE.MeshStandardMaterial({ color: jointColor, metalness: 0.5, roughness: 0.5 });
const linkMaterial = new THREE.MeshStandardMaterial({ color: linkColor, metalness: 0.5, roughness: 0.5 });
const tcpMaterial = new THREE.MeshStandardMaterial({ color: tcpColor, metalness: 0.5, roughness: 0.5 });

const fkLinkMaterial = new THREE.MeshStandardMaterial({ color: fkConfigColor, metalness: 0.5, roughness: 0.5, transparent: true, opacity: 0.8 });
const fkJointMaterial = new THREE.MeshStandardMaterial({ color: fkConfigColor, metalness: 0.5, roughness: 0.5, transparent: true, opacity: 0.8 });

const ikLinkMaterial = new THREE.MeshStandardMaterial({ color: ikConfigColor, metalness: 0.5, roughness: 0.5, transparent: true, opacity: 0.3 });
const ikJointMaterial = new THREE.MeshStandardMaterial({ color: ikConfigColor, metalness: 0.5, roughness: 0.5, transparent: true, opacity: 0.3 });

// Create geometries once
const linkGeometry = new THREE.CylinderGeometry(linkRadius, linkRadius, 1, 16); // Height 1, will be scaled
const jointGeometry = new THREE.SphereGeometry(jointRadius, 16, 16);
const tcpGeometry = new THREE.SphereGeometry(jointRadius * 1.2, 16, 16);


/**
 * Creates and adds a robot arm visualization to the scene.
 * Uses shared geometries and materials for efficiency.
 */
function drawRobot(points, T0_TCP, type = 'ik') {
    if (!points || points.length < 8) { // Expect Base + 6 Joints + TCP = 8 points
        console.error("Draw Error: Insufficient points provided.", points);
        return;
    }

    const group = new THREE.Group();
    const currentLinkMat = (type === 'fk') ? fkLinkMaterial : ikLinkMaterial;
    const currentJointMat = (type === 'fk') ? fkJointMaterial : ikJointMaterial;

    // Draw Links
    for (let i = 0; i < points.length - 1; i++) {
        const startPoint = points[i];
        const endPoint = points[i + 1];
        const distance = startPoint.distanceTo(endPoint);

        if (distance < 1e-4) continue;

        const linkCylinder = new THREE.Mesh(linkGeometry, currentLinkMat);
        linkCylinder.position.copy(startPoint).lerp(endPoint, 0.5);
        linkCylinder.scale.y = distance;
        linkCylinder.lookAt(endPoint);
        linkCylinder.rotateX(Math.PI / 2);
        group.add(linkCylinder);
    }

    // Draw Joints
    for (let i = 1; i < points.length -1; i++) {
         const jointSphere = new THREE.Mesh(jointGeometry, currentJointMat);
         jointSphere.position.copy(points[i]);
         group.add(jointSphere);
    }

    // Draw TCP Sphere
    const tcpSphere = new THREE.Mesh(tcpGeometry, tcpMaterial);
    tcpSphere.position.copy(points[points.length - 1]);
    group.add(tcpSphere);

    // Draw TCP Frame
    const tcpFrame = new THREE.AxesHelper(0.08);
    tcpFrame.matrixAutoUpdate = false;
    tcpFrame.matrix.copy(T0_TCP);
    group.add(tcpFrame);

    robotGroup.add(group);
}

/** Clear all previously drawn robot arms */
function clearRobots() {
    while (robotGroup.children.length > 0) {
        robotGroup.remove(robotGroup.children[0]);
    }
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
        drawRobot(fkResult.points, T_Target_TCP, 'fk');
        targetFrameHelper.matrixAutoUpdate = false;
        targetFrameHelper.matrix.copy(T_Target_TCP);
        targetFrameHelper.visible = true;
        fkSuccess = true;
    } else {
        console.error("FK failed for current slider angles:", sliderJointAngles.map(a=>(a*180/Math.PI).toFixed(1)));
        targetFrameHelper.visible = false;
    }

    // 2. Inverse Kinematics for the Target Pose
    let ikSolutions = [];
    let ikTime = 0;
    if (fkSuccess && T_Target_TCP) {
        const startTime = performance.now();
        try {
             // console.log("--- Calling IK ---");
             ikSolutions = inverseKinematics(T_Target_TCP);
             // console.log(`--- IK Returned ${ikSolutions.length} solutions ---`);
        } catch (e) {
            console.error("Error during Inverse Kinematics:", e);
            ikSolutions = [];
        }
        const endTime = performance.now();
        ikTime = endTime - startTime;

        // Draw IK solutions
        if (ikSolutions && ikSolutions.length > 0) {
            for (const solution of ikSolutions) {
                const ikAngles = solution[0];
                const isFkConfig = ikAngles.every((angle, i) => Math.abs(normalizeAngle(angle - sliderJointAngles[i])) < tol_compare);

                if (!isFkConfig) {
                    const ikFkResult = forwardKinematics(ikAngles);
                    if (ikFkResult) {
                        drawRobot(ikFkResult.points, ikFkResult.T0_TCP, 'ik');
                    } else {
                         console.warn("FK failed for a valid IK solution:", ikAngles.map(a=>(a*180/Math.PI).toFixed(1)));
                    }
                }
            }
        }
    }

    // 3. Update Info Display
    updateInfoDisplay(sliderJointAngles, T_Target_TCP, ikSolutions, ikTime);

}

/**
 * Updates the text display with current angles, FK pose, and IK info.
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
        if (loadingIndicator) {
             loadingIndicator.textContent = "Error";
             loadingIndicator.style.display = 'flex';
        }
    }
});
