/**
 * Interactive UR5e Kinematics Visualization using Three.js
 *
 * Based on the Python FK/IK logic provided.
 * Focuses on interactive Forward Kinematics (FK) visualization.
 * Allows users to control joint angles via sliders and see the 3D robot update.
 * Displays the calculated TCP pose.
 */

// Ensure Three.js and OrbitControls are loaded
if (typeof THREE === 'undefined') {
    console.error("THREE.js library not found. Make sure it's included before this script.");
}
// Note: OrbitControls attaches itself to the THREE namespace if loaded after three.min.js

// --- Constants and DH Parameters ---

const DH_PARAMS_UR5E = [
    { a: 0.0,    alpha: Math.PI / 2, d: 0.1625, theta_offset: 0.0 },
    { a: -0.425, alpha: 0.0,         d: 0.0,    theta_offset: 0.0 },
    { a: -0.3922,alpha: 0.0,         d: 0.0,    theta_offset: 0.0 },
    { a: 0.0,    alpha: Math.PI / 2, d: 0.1333, theta_offset: 0.0 },
    { a: 0.0,    alpha: -Math.PI / 2,d: 0.0997, theta_offset: 0.0 },
    { a: 0.0,    alpha: 0.0,         d: 0.0996, theta_offset: 0.0 }
];

const TCP_Z_OFFSET = 0.1565; // Standard TCP offset along flange Z

// Transformation from Flange frame (Link 6) to TCP frame
const H_FLANGE_TCP = new THREE.Matrix4().makeTranslation(0, 0, TCP_Z_OFFSET);

// --- Kinematics Functions (Adapted from Python) ---

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
 */
function forwardKinematics(jointAngles, dhParams = DH_PARAMS_UR5E, H_flange_tcp = H_FLANGE_TCP) {
    if (jointAngles.length !== dhParams.length) {
        throw new Error(`Angle count mismatch: Expected ${dhParams.length}, got ${jointAngles.length}`);
    }

    const transforms = [new THREE.Matrix4()]; // Start with identity matrix for base frame T0_0
    let T_prev = transforms[0].clone();

    for (let i = 0; i < dhParams.length; i++) {
        const p = dhParams[i];
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
        matrix.decompose(position, new THREE.Quaternion(), new THREE.Vector3());
        return position;
    });

    return { points, T0_TCP, transforms };
}

// --- Three.js Setup ---
let scene, camera, renderer, controls;
let robotGroup; // Group to hold all robot parts
let jointMeshes = []; // Store meshes for joints (spheres)
let linkMeshes = []; // Store meshes for links (cylinders)
let tcpHelper; // Axes helper for the TCP frame
const linkColors = [0x555555, 0xaaaaaa, 0x555555, 0xaaaaaa, 0x555555, 0xaaaaaa]; // Alternating link colors
const jointColor = 0x0055ff; // Blue for joints

const container = document.getElementById('kinematics-visualization');
if (!container) {
    console.error("Container element #kinematics-visualization not found.");
}

function initThreeJS() {
    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x2d3748); // Match container background

    // Camera
    const aspect = container.clientWidth / container.clientHeight;
    camera = new THREE.PerspectiveCamera(50, aspect, 0.1, 100);
    camera.position.set(1, 1.5, 2); // Adjusted initial camera position
    camera.lookAt(0, 0.4, 0); // Look towards the robot base area

    // Renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 10, 7);
    scene.add(directionalLight);

    // Controls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0.4, 0); // Set control target to robot base area
    controls.enableDamping = true; // Smooth camera movement
    controls.dampingFactor = 0.1;
    controls.screenSpacePanning = false; // Keep panning relative to target

    // Robot Group
    robotGroup = new THREE.Group();
    scene.add(robotGroup);

    // Coordinate Frame Helper (Base)
    const baseAxesHelper = new THREE.AxesHelper(0.2); // Smaller base helper
    scene.add(baseAxesHelper);

    // Ground Plane (Optional)
    const planeGeometry = new THREE.PlaneGeometry(3, 3);
    const planeMaterial = new THREE.MeshStandardMaterial({ color: 0x4a5568, side: THREE.DoubleSide });
    const plane = new THREE.Mesh(planeGeometry, planeMaterial);
    plane.rotation.x = -Math.PI / 2;
    plane.position.y = -0.01; // Slightly below base
    scene.add(plane);


    // Build Initial Robot Structure (will be updated)
    buildRobotModel();

    // Handle window resize
    window.addEventListener('resize', onWindowResize, false);

    // Start animation loop
    animate();
}

/**
 * Builds the initial geometric structure of the robot.
 * Positions will be updated by FK.
 */
function buildRobotModel() {
    // Clear previous meshes if any
    robotGroup.children.forEach(child => robotGroup.remove(child)); // Clear group
    jointMeshes = [];
    linkMeshes = [];

    const jointRadius = 0.04;
    const linkRadius = 0.03;
    const jointGeometry = new THREE.SphereGeometry(jointRadius, 16, 16);
    const jointMaterial = new THREE.MeshStandardMaterial({ color: jointColor });

    // Create joints (spheres) - 7 total (Base + 6 joints)
    for (let i = 0; i < 7; i++) {
        const joint = new THREE.Mesh(jointGeometry, jointMaterial);
        robotGroup.add(joint);
        jointMeshes.push(joint);
    }
    // Make base joint slightly larger
     jointMeshes[0].scale.set(1.5, 1.5, 1.5);


    // Create links (cylinders) - 6 total
    for (let i = 0; i < 6; i++) {
        const linkMaterial = new THREE.MeshStandardMaterial({ color: linkColors[i] });
        // Cylinder geometry will be created dynamically in updateRobotPose
        const link = new THREE.Mesh(undefined, linkMaterial); // Placeholder geometry
        robotGroup.add(link);
        linkMeshes.push(link);
    }

    // TCP Axes Helper
    tcpHelper = new THREE.AxesHelper(0.1); // Size of TCP frame axes
    robotGroup.add(tcpHelper);
}


/**
 * Updates the robot's visual pose based on FK results.
 * @param {object} fkResult - The result from forwardKinematics function.
 */
function updateRobotPose(fkResult) {
    const { points, transforms } = fkResult; // points contains Vector3 origins, transforms contains Matrix4

    if (points.length !== 8 || transforms.length !== 8) {
        console.error("FK result has unexpected length.");
        return;
    }

    // Update Joint Positions (Spheres) - points[0] is base, points[1] is J1 origin, ..., points[6] is J6 origin (flange)
    for (let i = 0; i < 7; i++) {
        if (jointMeshes[i] && points[i]) {
            jointMeshes[i].position.copy(points[i]);
        }
    }

    // Update Link Positions and Orientations (Cylinders)
    for (let i = 0; i < 6; i++) {
        const startPoint = points[i]; // Origin of frame i
        const endPoint = points[i + 1]; // Origin of frame i+1
        const linkMesh = linkMeshes[i];

        if (!startPoint || !endPoint || !linkMesh) continue;

        const distance = startPoint.distanceTo(endPoint);
        if (distance < 0.001) { // Avoid creating zero-length cylinders
             linkMesh.visible = false;
             continue;
        }
         linkMesh.visible = true;


        // Create or update cylinder geometry
        const linkRadius = 0.03;
        if (!linkMesh.geometry || linkMesh.geometry.parameters.height !== distance) {
             if(linkMesh.geometry) linkMesh.geometry.dispose(); // Dispose old geometry
            linkMesh.geometry = new THREE.CylinderGeometry(linkRadius, linkRadius, distance, 16);
        }

        // Position the cylinder halfway between the points
        linkMesh.position.copy(startPoint).lerp(endPoint, 0.5);

        // Orient the cylinder to point from startPoint to endPoint
        linkMesh.lookAt(endPoint);
        linkMesh.rotateX(Math.PI / 2); // Cylinders are oriented along Y by default, rotate to align with Z axis after lookAt
    }

    // Update TCP Helper Pose (using the T0_TCP matrix)
    const tcpTransform = transforms[7]; // Last transform is T0_TCP
    if (tcpHelper && tcpTransform) {
        const position = new THREE.Vector3();
        const quaternion = new THREE.Quaternion();
        const scale = new THREE.Vector3();
        tcpTransform.decompose(position, quaternion, scale);

        tcpHelper.position.copy(position);
        tcpHelper.quaternion.copy(quaternion);
    }
}


/**
 * Formats a THREE.Matrix4 for display.
 * @param {THREE.Matrix4} matrix - The matrix to format.
 * @returns {string} Formatted string representation.
 */
function formatMatrixForDisplay(matrix) {
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
        slider.addEventListener('input', handleSliderChange);
    } else {
        console.error(`Slider or display element not found for q${i}`);
    }
}
const tcpPoseDisplay = document.getElementById('tcp-pose-display');

function handleSliderChange() {
    const currentJointAngles = sliders.map(s => parseFloat(s.value));

    // Update value displays next to sliders
    currentJointAngles.forEach((angle, i) => {
        if (valueDisplays[i]) {
            valueDisplays[i].textContent = angle.toFixed(2);
        }
    });

    try {
        const fkResult = forwardKinematics(currentJointAngles);
        updateRobotPose(fkResult);

        // Display the calculated TCP Pose Matrix
        if (tcpPoseDisplay && fkResult.T0_TCP) {
            tcpPoseDisplay.innerHTML = `<strong>Calculated TCP Pose (T0_TCP):</strong>\n${formatMatrixForDisplay(fkResult.T0_TCP)}`;
        }

    } catch (error) {
        console.error("Error during FK or robot update:", error);
        if (tcpPoseDisplay) {
            tcpPoseDisplay.innerHTML = `<strong>Error calculating pose:</strong>\n${error.message}`;
        }
    }
}

// --- Animation Loop ---
function animate() {
    requestAnimationFrame(animate);
    controls.update(); // Only required if controls.enableDamping or autoRotate are set to true
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
if (container && typeof THREE !== 'undefined' && typeof THREE.OrbitControls !== 'undefined') {
    initThreeJS();
    // Initial update based on default slider values
    handleSliderChange();
} else {
     if(container) {
         container.innerHTML = '<p style="color: var(--text-muted); padding: 2rem; text-align: center;">Error: Could not initialize 3D visualization. Required libraries might be missing.</p>';
     }
}
