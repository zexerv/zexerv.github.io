import * as THREE from '../libs/three.min.js'; // Adjust path if needed
import { OrbitControls } from '../libs/OrbitControls.js'; // Adjust path if needed

// Ensure the kinematics solver is loaded (it exposes UR5eKinematics globally)
if (typeof UR5eKinematics === 'undefined') {
    throw new Error("UR5eKinematics solver script not loaded before interactive-kinematics.js");
}

// --- Scene Setup ---
const canvasContainer = document.getElementById('robot-canvas-container');
const canvas = document.getElementById('robot-canvas');
const loadingIndicator = document.getElementById('loading-indicator');
const ikStatusElement = document.getElementById('ik-status');

if (!canvasContainer || !canvas || !loadingIndicator || !ikStatusElement) {
    throw new Error("Required HTML elements not found for the visualization.");
}

loadingIndicator.style.display = 'block'; // Show loading

let scene, camera, renderer, controls;
let robotGroupFK, ikSolutionGroups = []; // Groups to hold robot parts
let targetTCPFrame; // Visual representation of the target TCP

// Colors and Materials
const fkColor = 0xff0000; // Red for FK
const ikColorPalette = [0x00ffff, 0x00ff00, 0xff00ff, 0xffff00, 0x0000ff, 0xffa500, 0x800080, 0x4682b4]; // Cyan, Green, Magenta, Yellow, Blue, Orange, Purple, SteelBlue
const jointRadius = 0.02;
const linkRadius = 0.015;

const fkMaterialSolid = new THREE.MeshLambertMaterial({ color: fkColor });
const fkLineMaterialSolid = new THREE.LineBasicMaterial({ color: fkColor, linewidth: 3 }); // Linewidth might not work on all systems

const ikMaterialsDashed = ikColorPalette.map(color =>
    new THREE.LineDashedMaterial({
        color: color,
        linewidth: 1.5, // Linewidth might not work on all systems
        scale: 1,
        dashSize: 0.03,
        gapSize: 0.02,
    })
);
const jointMaterial = new THREE.MeshLambertMaterial({ color: 0xaaaaaa }); // Grey joints
const baseMaterial = new THREE.MeshLambertMaterial({ color: 0x555555 });

// --- Robot Geometry Creation ---
const jointGeometry = new THREE.SphereGeometry(jointRadius, 16, 12);
// Base geometry (simple cylinder)
const baseGeometry = new THREE.CylinderGeometry(0.07, 0.07, 0.1, 32);
const baseMesh = new THREE.Mesh(baseGeometry, baseMaterial);
baseMesh.position.set(0, 0.05, 0); // Place base slightly above origin
baseMesh.rotation.x = -Math.PI / 2; // Align cylinder upright if needed


function createLinkMesh(p1, p2, material) {
    const vector = new THREE.Vector3().subVectors(p2, p1);
    const length = vector.length();
    if (length < 1e-6) return null; // Avoid zero-length links

    const geometry = new THREE.CylinderGeometry(linkRadius, linkRadius, length, 12);
    const mesh = new THREE.Mesh(geometry, material);

    // Position and Orient the Cylinder
    mesh.position.copy(p1).addScaledVector(vector, 0.5); // Center point
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), vector.normalize()); // Align Y axis along the vector

    return mesh;
}

function createLineSegments(points, material) {
    const geometry = new THREE.BufferGeometry().setFromPoints(points.map(p => new THREE.Vector3(p[0], p[1], p[2])));
    const line = new THREE.Line(geometry, material);
    if (material instanceof THREE.LineDashedMaterial) {
        line.computeLineDistances(); // Required for dashed lines
    }
    return line;
}


function createTargetFrame(size = 0.1) {
    const frameGroup = new THREE.Group();
    const origin = new THREE.Vector3(0, 0, 0);
    const x = new THREE.Vector3(1, 0, 0);
    const y = new THREE.Vector3(0, 1, 0);
    const z = new THREE.Vector3(0, 0, 1);

    frameGroup.add(new THREE.ArrowHelper(x, origin, size, 0xff0000, size * 0.2, size * 0.1)); // Red X
    frameGroup.add(new THREE.ArrowHelper(y, origin, size, 0x00ff00, size * 0.2, size * 0.1)); // Green Y
    frameGroup.add(new THREE.ArrowHelper(z, origin, size, 0x0000ff, size * 0.2, size * 0.1)); // Blue Z

    // Optional: Add a marker at the origin
    const sphereGeo = new THREE.SphereGeometry(size * 0.1, 8, 8);
    const sphereMat = new THREE.MeshBasicMaterial({ color: 0x000000 });
    const sphereMesh = new THREE.Mesh(sphereGeo, sphereMat);
    // frameGroup.add(sphereMesh); // Add if you want a black sphere at TCP

    frameGroup.visible = false; // Initially hidden
    return frameGroup;
}


// --- Initialization ---
function init() {
    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x282c34); // Dark background similar to theme

    // Camera
    const aspect = canvasContainer.clientWidth / canvasContainer.clientHeight;
    camera = new THREE.PerspectiveCamera(50, aspect, 0.1, 100);
    camera.position.set(1, 1, 1.5);
    camera.lookAt(0, 0.4, 0); // Look towards the robot base area

    // Renderer
    renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    renderer.setSize(canvasContainer.clientWidth, canvasContainer.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    // renderer.outputEncoding = THREE.sRGBEncoding; // Optional: Color space

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 10, 7);
    scene.add(directionalLight);

    // Controls
    controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0.4, 0); // Center controls roughly on robot
    controls.update();

    // Robot Group (FK)
    robotGroupFK = new THREE.Group();
    scene.add(robotGroupFK);
    scene.add(baseMesh); // Add static base

    // Target TCP Frame
    targetTCPFrame = createTargetFrame(0.15); // Make target frame larger
    scene.add(targetTCPFrame);


    // Initial Update
    updateVisualization();

    // Handle Resize
    window.addEventListener('resize', onWindowResize);

    loadingIndicator.style.display = 'none'; // Hide loading
    animate();
}

// --- Update Logic ---
let lastFKResult = null;

function updateVisualization() {
    // 1. Get joint angles from sliders
    const jointAngles = [];
    for (let i = 1; i <= 6; i++) {
        const slider = document.getElementById(`joint${i}-slider`);
        const valueSpan = document.getElementById(`joint${i}-value`);
        if (slider && valueSpan) {
            const angle = parseFloat(slider.value);
            jointAngles.push(angle);
            valueSpan.textContent = angle.toFixed(2);
        } else {
            console.error(`Slider or value span for joint ${i} not found!`);
            return; // Abort update
        }
    }

    // 2. Perform Forward Kinematics
    const fkResult = UR5eKinematics.forwardKinematics(jointAngles);
    lastFKResult = fkResult; // Store for IK

    // Clear previous FK robot visualization
    while (robotGroupFK.children.length > 0) {
        robotGroupFK.remove(robotGroupFK.children[0]);
    }
    // Clear previous IK visualizations
    ikSolutionGroups.forEach(group => scene.remove(group));
    ikSolutionGroups = [];
    ikStatusElement.textContent = "IK Solutions Found: -";
    targetTCPFrame.visible = false;


    if (fkResult && fkResult.points) {
        const points = fkResult.points.map(p => new THREE.Vector3(p[0], p[1], p[2])); // Convert to Vector3

        // Draw FK Robot (Solid Red Lines + Grey Joints)
        const fkLine = createLineSegments(fkResult.points, fkLineMaterialSolid);
        robotGroupFK.add(fkLine);

        points.forEach((point, index) => {
            if (index > 0 && index < points.length -1 ) { // Skip base point 0, skip TCP point
                const jointMesh = new THREE.Mesh(jointGeometry, jointMaterial);
                jointMesh.position.copy(point);
                robotGroupFK.add(jointMesh);
            }
        });

        // Update and show Target TCP Frame from FK result
        const T0_TCP_Matrix = new THREE.Matrix4();
        T0_TCP_Matrix.set( // Set from the 2D array row by row
            fkResult.T0_TCP[0][0], fkResult.T0_TCP[0][1], fkResult.T0_TCP[0][2], fkResult.T0_TCP[0][3],
            fkResult.T0_TCP[1][0], fkResult.T0_TCP[1][1], fkResult.T0_TCP[1][2], fkResult.T0_TCP[1][3],
            fkResult.T0_TCP[2][0], fkResult.T0_TCP[2][1], fkResult.T0_TCP[2][2], fkResult.T0_TCP[2][3],
            fkResult.T0_TCP[3][0], fkResult.T0_TCP[3][1], fkResult.T0_TCP[3][2], fkResult.T0_TCP[3][3]
        );
        targetTCPFrame.position.setFromMatrixPosition(T0_TCP_Matrix);
        targetTCPFrame.rotation.setFromRotationMatrix(T0_TCP_Matrix);
        targetTCPFrame.visible = true;


        // 3. Perform Inverse Kinematics using the FK result pose
        const ikSolutions = UR5eKinematics.inverseKinematics(fkResult.T0_TCP);
        ikStatusElement.textContent = `IK Solutions Found: ${ikSolutions.length}`;
        // console.log(`Found ${ikSolutions.length} IK solutions.`);


        // 4. Visualize IK Solutions (Dashed Lines)
        ikSolutions.forEach((ikAngles, solIndex) => {
            const ikFkResult = UR5eKinematics.forwardKinematics(ikAngles);
            if (ikFkResult && ikFkResult.points) {
                 // Check if this IK solution is very close to the FK solution
                let diffSumSq = 0;
                for(let j=0; j<6; ++j){
                    diffSumSq += (ikAngles[j] - jointAngles[j])**2;
                }

                // Only draw if it's significantly different from the FK pose
                if (Math.sqrt(diffSumSq) > 0.01) { // Threshold to avoid drawing over FK
                    const ikGroup = new THREE.Group();
                    const materialIndex = solIndex % ikMaterialsDashed.length;
                    const ikLine = createLineSegments(ikFkResult.points, ikMaterialsDashed[materialIndex]);

                     // We don't typically draw joints for IK solutions to reduce clutter
                     /*
                     ikFkResult.points.forEach((point, index) => {
                         if (index > 0 && index < ikFkResult.points.length - 1) { // Skip base and TCP
                             const jointMesh = new THREE.Mesh(jointGeometry, jointMaterial.clone()); // Clone material if needed
                             jointMesh.material.color.set(ikColorPalette[materialIndex]); // Color joints? Maybe keep grey.
                             jointMesh.material.transparent = true;
                             jointMesh.material.opacity = 0.6;
                             jointMesh.position.fromArray(point);
                             ikGroup.add(jointMesh);
                         }
                     });
                     */

                    ikGroup.add(ikLine);
                    scene.add(ikGroup);
                    ikSolutionGroups.push(ikGroup); // Keep track for clearing
                }
            } else {
                 console.warn(`FK check failed for IK solution index ${solIndex}`);
            }
        });

    } else {
        console.error("Forward Kinematics failed for the current slider angles.");
        ikStatusElement.textContent = "FK Failed";
        targetTCPFrame.visible = false;
    }
}

// --- Event Listeners ---
for (let i = 1; i <= 6; i++) {
    const slider = document.getElementById(`joint${i}-slider`);
    if (slider) {
        slider.addEventListener('input', updateVisualization);
    }
}

function onWindowResize() {
    const width = canvasContainer.clientWidth;
    const height = canvasContainer.clientHeight;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
}

// --- Animation Loop ---
function animate() {
    requestAnimationFrame(animate);
    controls.update(); // Only required if controls.enableDamping or enableZoom are set
    renderer.render(scene, camera);
}

// --- Start ---
init();