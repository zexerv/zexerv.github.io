// Import necessary Three.js components. Ensure these files are in the correct path.
import * as THREE from '../libs/three.min.js';
import { OrbitControls } from '../libs/OrbitControls.js';

// --- Pre-computation Checks ---
// Check if Three.js library is loaded correctly.
if (typeof THREE === 'undefined') {
    throw new Error("FATAL ERROR: THREE is not defined. Ensure three.min.js is loaded correctly before this script.");
}
// Check if OrbitControls is loaded correctly.
if (typeof OrbitControls === 'undefined') {
    throw new Error("FATAL ERROR: OrbitControls is not defined. Ensure OrbitControls.js is loaded correctly before this script and that this script has type='module'.");
}
// Check if the custom kinematics solver is loaded.
if (typeof UR5eKinematics === 'undefined') {
    throw new Error("FATAL ERROR: UR5eKinematics solver script not loaded before interactive-kinematics.js");
}

// --- DOM Element Checks ---
// Retrieve essential DOM elements for the visualization and controls.
const canvasContainer = document.getElementById('robot-canvas-container');
const canvas = document.getElementById('robot-canvas');
const loadingIndicator = document.getElementById('loading-indicator');
const ikStatusElement = document.getElementById('ik-status');

// Verify that all required HTML elements are present.
if (!canvasContainer || !canvas || !loadingIndicator || !ikStatusElement) {
    throw new Error("FATAL ERROR: Required HTML elements (canvas container, canvas, loading indicator, or IK status) not found. Check IDs in interactive-kinematics.html.");
}

// Show loading indicator while setting up.
loadingIndicator.style.display = 'block';

// --- Scene Setup Variables ---
let scene, camera, renderer, controls;
let robotGroupFK, ikSolutionGroups = []; // Groups to hold robot parts for FK and IK.
let targetTCPFrame; // Visual representation of the target TCP frame.

// --- Colors and Materials Definitions ---
// Define colors for FK and IK solutions for visual distinction.
const fkColor = 0xff0000; // Red for the primary FK robot.
const ikColorPalette = [0x00ffff, 0x00ff00, 0xff00ff, 0xffff00, 0x0000ff, 0xffa500, 0x800080, 0x4682b4]; // Palette for IK solutions.
// Define geometry parameters.
const jointRadius = 0.02;
const linkRadius = 0.015;

// Define materials for different robot parts.
const fkMaterialSolid = new THREE.MeshLambertMaterial({ color: fkColor }); // Solid material for FK links (currently unused, using lines).
const fkLineMaterialSolid = new THREE.LineBasicMaterial({ color: fkColor, linewidth: 3 }); // Solid line for FK robot links.

// Dashed line materials for IK solutions, cycling through the palette.
const ikMaterialsDashed = ikColorPalette.map(color =>
    new THREE.LineDashedMaterial({
        color: color,
        linewidth: 1.5, // Note: linewidth > 1 might not be supported on all systems/drivers.
        scale: 1,       // Controls the pattern repetition.
        dashSize: 0.03, // Length of dashes.
        gapSize: 0.02,  // Length of gaps.
    })
);
// Material for robot joints.
const jointMaterial = new THREE.MeshLambertMaterial({ color: 0xaaaaaa }); // Grey joints.
// Material for the robot base.
const baseMaterial = new THREE.MeshLambertMaterial({ color: 0x555555 }); // Dark grey base.

// --- Robot Geometry Creation ---
// Reusable geometry for joints (spheres).
const jointGeometry = new THREE.SphereGeometry(jointRadius, 16, 12);
// Geometry for the robot base (cylinder).
const baseGeometry = new THREE.CylinderGeometry(0.07, 0.07, 0.1, 32);
const baseMesh = new THREE.Mesh(baseGeometry, baseMaterial);
baseMesh.position.set(0, 0.05, 0); // Position the base slightly above the origin.
baseMesh.rotation.x = -Math.PI / 2; // Orient the cylinder upright if needed (depends on geometry definition).


/**
 * Creates a cylinder mesh representing a robot link between two points.
 * @param {THREE.Vector3} p1 - Start point of the link.
 * @param {THREE.Vector3} p2 - End point of the link.
 * @param {THREE.Material} material - The material for the link mesh.
 * @returns {THREE.Mesh | null} The link mesh or null if points are coincident.
 */
function createLinkMesh(p1, p2, material) {
    // Calculate the vector representing the link.
    const vector = new THREE.Vector3().subVectors(p2, p1);
    const length = vector.length();
    // Avoid creating zero-length cylinders.
    if (length < 1e-6) return null;

    // Create cylinder geometry.
    const geometry = new THREE.CylinderGeometry(linkRadius, linkRadius, length, 12);
    const mesh = new THREE.Mesh(geometry, material);

    // Position the cylinder halfway between the points.
    mesh.position.copy(p1).addScaledVector(vector, 0.5);
    // Orient the cylinder along the vector direction (aligns cylinder's Y-axis).
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), vector.normalize());

    return mesh;
}

/**
 * Creates line segments connecting a series of points.
 * @param {number[][]} pointsArray - Array of points, where each point is [x, y, z].
 * @param {THREE.LineBasicMaterial | THREE.LineDashedMaterial} material - The material for the line.
 * @returns {THREE.Line} The created line object.
 */
function createLineSegments(pointsArray, material) {
    // Convert array of arrays to array of Vector3.
    const points = pointsArray.map(p => new THREE.Vector3(p[0], p[1], p[2]));
    // Create geometry from points.
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    // Create the line object.
    const line = new THREE.Line(geometry, material);
    // Required for dashed lines to render correctly.
    if (material instanceof THREE.LineDashedMaterial) {
        line.computeLineDistances();
    }
    return line;
}

/**
 * Creates a visual representation of a coordinate frame (X, Y, Z axes).
 * @param {number} [size=0.1] - The length of the axis arrows.
 * @returns {THREE.Group} A group containing the axis arrows.
 */
function createTargetFrame(size = 0.1) {
    const frameGroup = new THREE.Group();
    const origin = new THREE.Vector3(0, 0, 0);
    // Define axis vectors.
    const x = new THREE.Vector3(1, 0, 0);
    const y = new THREE.Vector3(0, 1, 0);
    const z = new THREE.Vector3(0, 0, 1);

    // Create arrows for each axis with distinct colors.
    frameGroup.add(new THREE.ArrowHelper(x, origin, size, 0xff0000, size * 0.2, size * 0.1)); // Red X
    frameGroup.add(new THREE.ArrowHelper(y, origin, size, 0x00ff00, size * 0.2, size * 0.1)); // Green Y
    frameGroup.add(new THREE.ArrowHelper(z, origin, size, 0x0000ff, size * 0.2, size * 0.1)); // Blue Z

    // Initially hide the frame.
    frameGroup.visible = false;
    return frameGroup;
}


// --- Initialization Function ---
/**
 * Initializes the Three.js scene, camera, renderer, controls, and initial robot state.
 */
function init() {
    try {
        // Scene: Container for all 3D objects.
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x282c34); // Dark background.

        // Camera: Defines the viewpoint.
        const aspect = canvasContainer.clientWidth / canvasContainer.clientHeight;
        camera = new THREE.PerspectiveCamera(50, aspect, 0.1, 100); // fov, aspect, near, far
        camera.position.set(1, 1, 1.5); // Set initial camera position.
        camera.lookAt(0, 0.4, 0); // Point camera towards the robot base area.

        // Renderer: Draws the scene onto the canvas.
        renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
        renderer.setSize(canvasContainer.clientWidth, canvasContainer.clientHeight);
        renderer.setPixelRatio(window.devicePixelRatio); // Adjust for high-DPI screens.

        // Lighting: Illuminate the scene.
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6); // Soft ambient light.
        scene.add(ambientLight);
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8); // Brighter directional light.
        directionalLight.position.set(5, 10, 7); // Position the light source.
        scene.add(directionalLight);

        // Controls: Allow user interaction (orbit, zoom, pan).
        controls = new OrbitControls(camera, renderer.domElement);
        controls.target.set(0, 0.4, 0); // Set the point around which the camera orbits.
        controls.enableDamping = true; // Add smooth camera movement inertia.
        controls.dampingFactor = 0.1;
        controls.update(); // Apply initial target setting.

        // Robot Group (FK): Group to hold the main robot visualization.
        robotGroupFK = new THREE.Group();
        scene.add(robotGroupFK);
        // Add the static base mesh to the scene.
        scene.add(baseMesh);

        // Target TCP Frame: Visual indicator for the FK-derived TCP pose.
        targetTCPFrame = createTargetFrame(0.15); // Create a larger frame for visibility.
        scene.add(targetTCPFrame);

        // Perform the initial visualization update based on default slider values.
        updateVisualization();

        // Add event listener for window resize to keep the visualization responsive.
        window.addEventListener('resize', onWindowResize);

        // Hide loading indicator now that setup is complete.
        loadingIndicator.style.display = 'none';
        // Start the animation loop.
        animate();

    } catch (error) {
        // Catch any errors during initialization and report them.
        console.error("Error during Three.js initialization:", error);
        loadingIndicator.textContent = "Error initializing visualization. Check console.";
        loadingIndicator.style.color = "red";
        loadingIndicator.style.display = 'block';
    }
}

// --- Update Logic ---
let lastFKResult = null; // Store the last FK result to feed into IK.

/**
 * Updates the robot visualization based on current slider values.
 * Performs FK, updates the main robot display, performs IK, and displays solutions.
 */
function updateVisualization() {
    // 1. Get current joint angles from sliders
    const jointAngles = [];
    let slidersFound = true;
    for (let i = 1; i <= 6; i++) {
        const slider = document.getElementById(`joint${i}-slider`);
        const valueSpan = document.getElementById(`joint${i}-value`);
        if (slider && valueSpan) {
            const angle = parseFloat(slider.value);
            jointAngles.push(angle);
            // Update the text display next to the slider.
            valueSpan.textContent = angle.toFixed(2);
        } else {
            console.error(`Slider or value span for joint ${i} not found! Cannot update.`);
            slidersFound = false;
            break; // Stop processing if a slider is missing.
        }
    }
    // Abort if sliders weren't found.
    if (!slidersFound) return;

    // 2. Perform Forward Kinematics
    try {
        lastFKResult = UR5eKinematics.forwardKinematics(jointAngles);
    } catch (error) {
        console.error("Error during Forward Kinematics calculation:", error);
        lastFKResult = null; // Ensure FK result is null on error.
    }


    // --- Clear Previous Visualizations ---
    // Clear previous FK robot parts (lines, joints).
    while (robotGroupFK.children.length > 0) {
        const child = robotGroupFK.children[0];
        robotGroupFK.remove(child);
        // Dispose geometry and material if necessary to free memory (optional but good practice).
        if (child.geometry) child.geometry.dispose();
        if (child.material) child.material.dispose();
    }
    // Clear previous IK solution groups.
    ikSolutionGroups.forEach(group => {
        while (group.children.length > 0) {
            const child = group.children[0];
            group.remove(child);
            if (child.geometry) child.geometry.dispose();
            if (child.material) child.material.dispose();
        }
        scene.remove(group); // Remove the group itself from the scene.
    });
    ikSolutionGroups = []; // Reset the array.
    // Reset status text and hide target frame initially.
    ikStatusElement.textContent = "Calculating...";
    targetTCPFrame.visible = false;

    // --- Draw New Visualizations ---
    // Proceed only if FK was successful.
    if (lastFKResult && lastFKResult.points && lastFKResult.T0_TCP) {
        // Draw FK Robot (Solid Red Lines + Grey Joints)
        try {
            // Create and add the line segments for the FK robot arm.
            const fkLine = createLineSegments(lastFKResult.points, fkLineMaterialSolid);
            robotGroupFK.add(fkLine);

            // Add spheres to represent joints (excluding base and TCP).
            lastFKResult.points.forEach((point, index) => {
                if (index > 0 && index < lastFKResult.points.length - 1) {
                    const jointMesh = new THREE.Mesh(jointGeometry, jointMaterial);
                    jointMesh.position.fromArray(point); // Set position from array [x, y, z].
                    robotGroupFK.add(jointMesh);
                }
            });

            // Update and show Target TCP Frame based on FK result matrix.
            const T0_TCP_Matrix = new THREE.Matrix4();
            // Set the Three.js Matrix4 from the 2D array returned by FK.
            T0_TCP_Matrix.set(
                lastFKResult.T0_TCP[0][0], lastFKResult.T0_TCP[0][1], lastFKResult.T0_TCP[0][2], lastFKResult.T0_TCP[0][3],
                lastFKResult.T0_TCP[1][0], lastFKResult.T0_TCP[1][1], lastFKResult.T0_TCP[1][2], lastFKResult.T0_TCP[1][3],
                lastFKResult.T0_TCP[2][0], lastFKResult.T0_TCP[2][1], lastFKResult.T0_TCP[2][2], lastFKResult.T0_TCP[2][3],
                lastFKResult.T0_TCP[3][0], lastFKResult.T0_TCP[3][1], lastFKResult.T0_TCP[3][2], lastFKResult.T0_TCP[3][3]
            );
            // Apply the position and rotation from the matrix to the target frame object.
            targetTCPFrame.position.setFromMatrixPosition(T0_TCP_Matrix);
            targetTCPFrame.rotation.setFromRotationMatrix(T0_TCP_Matrix);
            targetTCPFrame.visible = true; // Make the frame visible.

        } catch (error) {
            console.error("Error drawing FK robot:", error);
            // Optionally clear the partially drawn FK robot here.
        }


        // 3. Perform Inverse Kinematics using the FK result pose.
        let ikSolutions = [];
        try {
            ikSolutions = UR5eKinematics.inverseKinematics(lastFKResult.T0_TCP);
            ikStatusElement.textContent = `IK Solutions Found: ${ikSolutions.length}`;
        } catch (error) {
            console.error("Error during Inverse Kinematics calculation:", error);
            ikStatusElement.textContent = "IK Error";
            ikSolutions = []; // Ensure solutions array is empty on error.
        }

        // 4. Visualize IK Solutions (Dashed Lines)
        ikSolutions.forEach((ikAngles, solIndex) => {
            try {
                // Perform FK check for the IK solution to get its points.
                const ikFkResult = UR5eKinematics.forwardKinematics(ikAngles);
                if (ikFkResult && ikFkResult.points) {
                    // Check if this IK solution is numerically very close to the current FK slider configuration.
                    let diffSumSq = 0;
                    for (let j = 0; j < 6; ++j) {
                        // Normalize angle difference to handle wrapping around +/- PI
                        let angleDiff = ikAngles[j] - jointAngles[j];
                        angleDiff = Math.atan2(Math.sin(angleDiff), Math.cos(angleDiff));
                        diffSumSq += angleDiff * angleDiff;
                    }

                    // Only draw the IK solution if it's significantly different from the FK pose shown in red.
                    // This avoids clutter by not drawing an identical pose on top.
                    const angleDifferenceThreshold = 0.01; // Radians threshold (approx 0.5 degrees)
                    if (Math.sqrt(diffSumSq) > angleDifferenceThreshold) {
                        const ikGroup = new THREE.Group(); // Group for this specific IK solution.
                        // Select material from the palette, cycling if more solutions than colors.
                        const materialIndex = solIndex % ikMaterialsDashed.length;
                        const ikLine = createLineSegments(ikFkResult.points, ikMaterialsDashed[materialIndex]);

                        // Add the dashed line representing the IK solution arm.
                        ikGroup.add(ikLine);
                        // Add the group to the main scene.
                        scene.add(ikGroup);
                        // Keep track of the group so it can be cleared later.
                        ikSolutionGroups.push(ikGroup);
                    }
                } else {
                    // Log if FK check failed for a specific IK solution.
                    console.warn(`FK check failed for IK solution index ${solIndex}. Angles:`, ikAngles.map(a => a.toFixed(3)));
                }
            } catch (error) {
                console.error(`Error processing/drawing IK solution ${solIndex}:`, error);
            }
        });

    } else {
        // Handle case where initial FK failed.
        console.error("Forward Kinematics failed for the current slider angles. Cannot perform IK or draw robot.");
        ikStatusElement.textContent = "FK Failed";
        targetTCPFrame.visible = false; // Ensure target frame is hidden.
    }
}

// --- Event Listeners ---
// Attach the update function to the 'input' event of each slider.
// 'input' fires continuously as the slider moves.
for (let i = 1; i <= 6; i++) {
    const slider = document.getElementById(`joint${i}-slider`);
    if (slider) {
        slider.addEventListener('input', updateVisualization);
    }
}

/**
 * Handles window resize events to adjust camera aspect ratio and renderer size.
 */
function onWindowResize() {
    const width = canvasContainer.clientWidth;
    const height = canvasContainer.clientHeight;

    // Update camera aspect ratio.
    camera.aspect = width / height;
    camera.updateProjectionMatrix();

    // Update renderer size.
    renderer.setSize(width, height);
}

// --- Animation Loop ---
/**
 * The main animation loop function called repeatedly via requestAnimationFrame.
 */
function animate() {
    // Schedule the next frame.
    requestAnimationFrame(animate);
    // Update orbit controls (needed for damping).
    controls.update();
    // Render the scene from the camera's perspective.
    renderer.render(scene, camera);
}

// --- Start ---
// Initialize the visualization.
init();
