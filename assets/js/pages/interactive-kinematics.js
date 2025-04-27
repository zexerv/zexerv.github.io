import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GUI } from 'three/addons/libs/lil-gui.module.min.js'; // Optional: for debugging

// --- Configuration & Constants ---
const DH_PARAMS_UR5E = [
    { a: 0.0, alpha: Math.PI / 2, d: 0.1625, theta_offset: 0.0 },
    { a: -0.425, alpha: 0.0, d: 0.0, theta_offset: 0.0 },
    { a: -0.3922, alpha: 0.0, d: 0.0, theta_offset: 0.0 },
    { a: 0.0, alpha: Math.PI / 2, d: 0.1333, theta_offset: 0.0 },
    { a: 0.0, alpha: -Math.PI / 2, d: 0.0997, theta_offset: 0.0 },
    { a: 0.0, alpha: 0.0, d: 0.0996, theta_offset: 0.0 }
];

const TCP_Z_OFFSET = 0.1565; // As defined in Python solver

const H_FLANGE_TCP = new THREE.Matrix4().makeTranslation(0, 0, TCP_Z_OFFSET);

const JOINT_COLORS = [0xff0000, 0x00ff00, 0x0000ff, 0xffff00, 0xff00ff, 0x00ffff]; // Color per joint
const LINK_COLOR = 0xaaaaaa;
const TCP_COLOR = 0xff8800;
const AXIS_LENGTH = 0.05; // For TCP frame axes
const JOINT_RADIUS = 0.03;
const LINK_RADIUS = 0.02;

const initialJointAngles = [0.0, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, -Math.PI / 2, 0.0]; // Radians

// --- DOM Elements ---
const canvasContainer = document.getElementById('kinematics-canvas-container');
const canvas = document.getElementById('kinematics-canvas');
const slidersContainer = document.getElementById('joint-sliders');
const tcpPoseOutput = document.getElementById('tcp-pose-output');
const loadingMessage = document.getElementById('loading-message');
const errorMessage = document.getElementById('error-message');

// --- Global Three.js Variables ---
let scene, camera, renderer, controls;
let robotGroup; // Group to hold all robot parts
let jointMeshes = [];
let linkMeshes = [];
let tcpFrameHelper;
let jointSliders = [];
let valueDisplays = [];

// --- Helper Functions ---

// DH Matrix function (Adapted from Python)
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

// Forward Kinematics function (Adapted from Python)
function forwardKinematics(jointAngles) {
    if (jointAngles.length !== DH_PARAMS_UR5E.length) {
        console.error("Angle count mismatch");
        return { points: [], T0_TCP: null, T0_Flange: null };
    }

    let transforms = [new THREE.Matrix4()]; // Start with identity matrix at base
    let T_prev = transforms[0].clone();

    for (let i = 0; i < DH_PARAMS_UR5E.length; i++) {
        const p = DH_PARAMS_UR5E[i];
        const theta = jointAngles[i] + p.theta_offset;
        const T_i_minus_1_to_i = dhMatrix(p.a, p.alpha, p.d, theta);
        const T_curr = new THREE.Matrix4().multiplyMatrices(T_prev, T_i_minus_1_to_i);
        transforms.push(T_curr.clone());
        T_prev = T_curr;
    }

    const T0_Flange = transforms[transforms.length - 1]; // Last calculated matrix is T0_Flange
    const T0_TCP = new THREE.Matrix4().multiplyMatrices(T0_Flange, H_FLANGE_TCP);

    // Add TCP transform for point extraction
    transforms.push(T0_TCP.clone());

    // Extract points from transformation matrices
    const points = transforms.map(T => {
        const position = new THREE.Vector3();
        T.decompose(position, new THREE.Quaternion(), new THREE.Vector3());
        return position;
    });

    return { points, T0_TCP, T0_Flange, transforms }; // Also return all transforms
}

function formatMatrix(matrix) {
    const e = matrix.elements;
    let str = "";
    for (let i = 0; i < 4; i++) {
        str += `[ ${e[i].toFixed(3)}, ${e[i + 4].toFixed(3)}, ${e[i + 8].toFixed(3)}, ${e[i + 12].toFixed(3)} ]\n`;
    }
    return str.trim();
}

function createAxis(color, length = 0.1) {
    const material = new THREE.LineBasicMaterial({ color: color, linewidth: 2 });
    const points = [new THREE.Vector3(0, 0, 0), new THREE.Vector3(length, 0, 0)]; // Default X axis
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    return new THREE.Line(geometry, material);
}

// --- Initialization ---
function init() {
    try {
        // Scene
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x282c34); // Match container background

        // Camera
        const aspect = canvasContainer.clientWidth / canvasContainer.clientHeight;
        camera = new THREE.PerspectiveCamera(60, aspect, 0.1, 100);
        camera.position.set(0.8, 0.8, 1.2); // Adjusted starting position
        camera.lookAt(0, 0, 0.3); // Look towards the base/lower part

        // Renderer
        renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
        renderer.setSize(canvasContainer.clientWidth, canvasContainer.clientHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        // renderer.outputEncoding = THREE.sRGBEncoding; // Deprecated, use colorManagement
        THREE.ColorManagement.enabled = true;


        // Lights
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        scene.add(ambientLight);
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(5, 10, 7.5);
        scene.add(directionalLight);

        // Controls
        controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.1;
        controls.screenSpacePanning = false;
        controls.target.set(0, 0, 0.3); // Set control target

        // Robot Group
        robotGroup = new THREE.Group();
        scene.add(robotGroup);

        // Coordinate Axes Helper (at origin)
        const axesHelper = new THREE.AxesHelper(0.3);
        scene.add(axesHelper);

        // Ground Plane (Optional)
        const planeGeometry = new THREE.PlaneGeometry(2, 2);
        const planeMaterial = new THREE.MeshStandardMaterial({ color: 0x444444, side: THREE.DoubleSide });
        const plane = new THREE.Mesh(planeGeometry, planeMaterial);
        plane.rotation.x = -Math.PI / 2; // Rotate to be horizontal
        scene.add(plane);


        // Create Sliders
        createSliders();

        // Create Robot Geometry (initial state)
        createRobotGeometry();
        updateRobotPose(initialJointAngles); // Initial pose calculation

        // Hide loading message
        loadingMessage.style.display = 'none';

        // Start animation loop
        animate();

    } catch (error) {
        console.error("Initialization failed:", error);
        loadingMessage.style.display = 'none';
        errorMessage.textContent = `Error initializing 3D view: ${error.message}`;
        errorMessage.style.display = 'block';
    }
}

function createSliders() {
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
        slider.addEventListener('input', onSliderChange);
    }
}


function createRobotGeometry() {
    robotGroup.clear(); // Remove previous meshes
    jointMeshes = [];
    linkMeshes = [];

    // Base Joint (Sphere)
    const baseMaterial = new THREE.MeshStandardMaterial({ color: 0x555555 });
    const baseGeometry = new THREE.SphereGeometry(JOINT_RADIUS * 1.5, 32, 16);
    const baseMesh = new THREE.Mesh(baseGeometry, baseMaterial);
    robotGroup.add(baseMesh); // Positioned at origin by default

    // Create Joints (Spheres) and Links (Cylinders)
    const jointMaterial = new THREE.MeshStandardMaterial({ color: 0xdddddd }); // Placeholder color
    const linkMaterial = new THREE.MeshStandardMaterial({ color: LINK_COLOR, roughness: 0.5, metalness: 0.2 });
    const jointGeometry = new THREE.SphereGeometry(JOINT_RADIUS, 16, 12);

    for (let i = 0; i < 6; i++) {
        // Joint Sphere
        const jointMesh = new THREE.Mesh(jointGeometry.clone(), jointMaterial.clone());
        jointMesh.material.color.setHex(JOINT_COLORS[i]); // Set specific joint color
        robotGroup.add(jointMesh);
        jointMeshes.push(jointMesh);

        // Link Cylinder (will be positioned/oriented in update)
        // Use a basic cylinder geometry; length/orientation determined by FK points
        const linkGeometry = new THREE.CylinderGeometry(LINK_RADIUS, LINK_RADIUS, 1, 8); // Length=1 initially
        const linkMesh = new THREE.Mesh(linkGeometry, linkMaterial.clone());
        robotGroup.add(linkMesh);
        linkMeshes.push(linkMesh);
    }

     // TCP Frame Helper (Axes)
     tcpFrameHelper = new THREE.AxesHelper(AXIS_LENGTH * 2); // Make TCP axes slightly larger
     robotGroup.add(tcpFrameHelper);
}

// --- Update Logic ---
function updateRobotPose(jointAngles) {
    const { points, T0_TCP, transforms } = forwardKinematics(jointAngles);

    if (!points || points.length < 8 || !T0_TCP) { // Need points for Base, J1-J6, TCP
        console.error("FK failed or returned insufficient points.");
        tcpPoseOutput.textContent = "Error in FK calculation.";
        return;
    }

     // Update Joint Positions (Spheres: J1 to J6)
     // points[0] = Base, points[1]=J1, ..., points[6]=J6, points[7]=TCP
     for (let i = 0; i < 6; i++) {
        if (jointMeshes[i] && points[i + 1]) {
             jointMeshes[i].position.copy(points[i + 1]);
        }
     }

     // Update Link Positions and Orientations (Cylinders)
     // Link 0 (Base to J1), Link 1 (J1 to J2), ..., Link 5 (J5 to J6)
     for (let i = 0; i < 6; i++) {
         if (linkMeshes[i] && points[i] && points[i + 1]) {
             const startPoint = points[i];
             const endPoint = points[i + 1];
             const direction = new THREE.Vector3().subVectors(endPoint, startPoint);
             const length = direction.length();
             const midPoint = new THREE.Vector3().addVectors(startPoint, endPoint).multiplyScalar(0.5);

             if (length > 0.001) { // Avoid issues with zero-length links
                 linkMeshes[i].scale.set(1, length, 1); // Scale cylinder height
                 linkMeshes[i].position.copy(midPoint);

                 // Align cylinder's Y-axis with the direction vector
                 const quaternion = new THREE.Quaternion();
                 const cylinderUp = new THREE.Vector3(0, 1, 0); // Default cylinder orientation
                 quaternion.setFromUnitVectors(cylinderUp, direction.normalize());
                 linkMeshes[i].setRotationFromQuaternion(quaternion);
                 linkMeshes[i].visible = true;
             } else {
                 linkMeshes[i].visible = false; // Hide zero-length links
             }
         } else {
             if(linkMeshes[i]) linkMeshes[i].visible = false;
         }
     }

     // Update TCP Frame Helper Position and Orientation
     if (tcpFrameHelper && T0_TCP) {
         const position = new THREE.Vector3();
         const quaternion = new THREE.Quaternion();
         const scale = new THREE.Vector3();
         T0_TCP.decompose(position, quaternion, scale);

         tcpFrameHelper.position.copy(position);
         tcpFrameHelper.setRotationFromQuaternion(quaternion);
     }

    // Update TCP Pose display
    if (T0_TCP) {
        tcpPoseOutput.textContent = formatMatrix(T0_TCP);
    } else {
        tcpPoseOutput.textContent = "N/A";
    }
}

function onSliderChange() {
    const currentJointAngles = jointSliders.map(slider => parseFloat(slider.value));
    // Update value displays next to sliders
    currentJointAngles.forEach((val, i) => {
        if (valueDisplays[i]) {
            valueDisplays[i].textContent = val.toFixed(2);
        }
    });
    updateRobotPose(currentJointAngles);
}

// --- Animation Loop ---
function animate() {
    requestAnimationFrame(animate);
    controls.update(); // Only required if controls.enableDamping or .autoRotate are set to true
    renderer.render(scene, camera);
}

// --- Resize Handling ---
function onWindowResize() {
    const width = canvasContainer.clientWidth;
    const height = canvasContainer.clientHeight;

    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
}

// --- Start ---
window.addEventListener('resize', onWindowResize);
init(); // Initialize everything