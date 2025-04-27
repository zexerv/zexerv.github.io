// Ensure the DOM is fully loaded before running the script
document.addEventListener('DOMContentLoaded', () => {

    // --- Constants and Configuration ---
    const DH_PARAMS_UR5E = [
        // DH parameters for UR5e (a, alpha, d, theta_offset)
        // Note: three.js uses Y-up by default, ROS/robotics often use Z-up.
        // We'll stick to standard DH conventions and adjust visualization accordingly.
        { a: 0.0,     alpha: Math.PI / 2, d: 0.1625, offset: 0.0 }, // Base to Joint 1
        { a: -0.425,  alpha: 0.0,         d: 0.0,    offset: 0.0 }, // Joint 1 to Joint 2
        { a: -0.3922, alpha: 0.0,         d: 0.0,    offset: 0.0 }, // Joint 2 to Joint 3
        { a: 0.0,     alpha: Math.PI / 2, d: 0.1333, offset: 0.0 }, // Joint 3 to Joint 4
        { a: 0.0,     alpha: -Math.PI / 2,d: 0.0997, offset: 0.0 }, // Joint 4 to Joint 5
        { a: 0.0,     alpha: 0.0,         d: 0.0996, offset: 0.0 }  // Joint 5 to Joint 6 (Flange)
    ];

    // Tool Center Point (TCP) offset from the flange (Joint 6)
    // Assuming TCP is offset only along the Z-axis of the flange frame
    const TCP_Z_OFFSET = 0.1565; // Example offset, adjust if needed
    const H_FLANGE_TCP = new THREE.Matrix4().makeTranslation(0, 0, TCP_Z_OFFSET);

    // Initial joint angles (matches the HTML slider defaults)
    const initialJointAngles = [
        0.0, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, -Math.PI / 2, 0.0
    ];

    // Colors for robot parts
    const BASE_COLOR = 0x666666; // Dark grey
    const LINK_COLOR = 0xAAAAAA; // Light grey
    const JOINT_COLOR = 0x0077CC; // Blue
    const TCP_COLOR = 0xFF0000;   // Red

    // --- three.js Scene Setup ---
    const container = document.getElementById('kinematics-canvas-container');
    if (!container) {
        console.error("Canvas container not found!");
        return;
    }

    let scene, camera, renderer, controls;
    let robotBase, jointMeshes = [], linkMeshes = [], tcpMesh;
    let jointFrames = []; // To store THREE.Object3D representing each joint frame

    // --- Kinematics Functions (Ported from Python) ---

    /**
     * Calculates the Denavit-Hartenberg transformation matrix.
     * @param {number} a - Link length
     * @param {number} alpha - Link twist
     * @param {number} d - Link offset
     * @param {number} theta - Joint angle
     * @returns {THREE.Matrix4} Transformation matrix
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
     * Calculates the forward kinematics for the given joint angles.
     * @param {number[]} jointAngles - Array of 6 joint angles in radians.
     * @returns {{transforms: THREE.Matrix4[], T0_TCP: THREE.Matrix4}} Object containing transforms of each frame and the final TCP transform.
     */
    function forwardKinematics(jointAngles) {
        if (jointAngles.length !== DH_PARAMS_UR5E.length) {
            throw new Error(`Angle count mismatch: Expected ${DH_PARAMS_UR5E.length}, got ${jointAngles.length}`);
        }

        const transforms = [new THREE.Matrix4()]; // Start with identity matrix for base frame T0_0
        let T_prev = transforms[0].clone();

        for (let i = 0; i < DH_PARAMS_UR5E.length; i++) {
            const p = DH_PARAMS_UR5E[i];
            const theta = jointAngles[i] + p.offset;
            const T_i_minus_1_to_i = dhMatrix(p.a, p.alpha, p.d, theta);

            // Calculate T0_i = T0_{i-1} * T_{i-1}_i
            const T_curr = new THREE.Matrix4().multiplyMatrices(T_prev, T_i_minus_1_to_i);
            transforms.push(T_curr);
            T_prev = T_curr; // Update for next iteration
        }

        const T0_flange = transforms[transforms.length - 1]; // Last calculated transform is T0_6 (flange)
        const T0_TCP = new THREE.Matrix4().multiplyMatrices(T0_flange, H_FLANGE_TCP);

        return { transforms, T0_TCP };
    }

    // --- Robot Visualization Functions ---

    /**
     * Creates the visual representation of the robot base.
     */
    function createRobotBase() {
        const baseGeometry = new THREE.CylinderGeometry(0.1, 0.1, 0.05, 32);
        const baseMaterial = new THREE.MeshStandardMaterial({ color: BASE_COLOR });
        robotBase = new THREE.Mesh(baseGeometry, baseMaterial);
        robotBase.position.y = 0.025; // Position base slightly above the grid
        robotBase.castShadow = true;
        robotBase.receiveShadow = true;
        scene.add(robotBase);

        // Create the first frame (base frame)
        const baseFrame = new THREE.Object3D();
        scene.add(baseFrame);
        jointFrames.push(baseFrame);
    }

    /**
     * Creates the visual representation of the robot joints and links.
     */
    function createRobotGeometry() {
        jointMeshes = [];
        linkMeshes = [];
        jointFrames = [jointFrames[0]]; // Keep the base frame

        const jointGeometry = new THREE.SphereGeometry(0.04, 16, 16); // Smaller joints
        const jointMaterial = new THREE.MeshStandardMaterial({ color: JOINT_COLOR, metalness: 0.3, roughness: 0.6 });

        const linkMaterial = new THREE.MeshStandardMaterial({ color: LINK_COLOR, metalness: 0.5, roughness: 0.5 });

        let parentFrame = jointFrames[0]; // Start with the base frame

        for (let i = 0; i < DH_PARAMS_UR5E.length; i++) {
            // Create a frame (Object3D) for this joint, positioned relative to the previous one
            const frame = new THREE.Object3D();
            parentFrame.add(frame); // Add to the previous frame
            jointFrames.push(frame);

            // Create joint mesh (sphere) - positioned at the origin of its frame
            const jointMesh = new THREE.Mesh(jointGeometry, jointMaterial);
            jointMesh.castShadow = true;
            frame.add(jointMesh); // Add joint sphere to its frame
            jointMeshes.push(jointMesh);

            // Create link mesh (cylinder) connecting previous joint to this one
            // This requires calculating the position of the current joint relative to the previous one
            // For simplicity, we'll update link positions/rotations in the update function
            // Here, just create basic cylinders that we'll transform later
            const linkGeometry = new THREE.CylinderGeometry(0.03, 0.03, 1, 16); // Placeholder length 1
            const linkMesh = new THREE.Mesh(linkGeometry, linkMaterial);
            linkMesh.castShadow = true;
            linkMesh.receiveShadow = true;
            // Add link relative to the *previous* joint's frame initially
            jointFrames[i].add(linkMesh); // Add to the frame of the joint *before* this link
            linkMeshes.push(linkMesh);

            parentFrame = frame; // Update parent for the next iteration
        }

        // Create TCP mesh
        const tcpGeometry = new THREE.SphereGeometry(0.02, 16, 16);
        const tcpMaterial = new THREE.MeshStandardMaterial({ color: TCP_COLOR, emissive: TCP_COLOR, emissiveIntensity: 0.5 });
        tcpMesh = new THREE.Mesh(tcpGeometry, tcpMaterial);
        tcpMesh.castShadow = true;
        // Add TCP mesh relative to the last joint frame (flange) initially
        jointFrames[jointFrames.length - 1].add(tcpMesh);

        // Add coordinate axes helper to the TCP for orientation
        const tcpAxesHelper = new THREE.AxesHelper(0.05);
        tcpMesh.add(tcpAxesHelper); // Add axes relative to the TCP sphere

        // Initial update to position everything correctly
        updateRobotPose(initialJointAngles);
    }

    /**
     * Updates the robot's visual pose based on joint angles.
     * @param {number[]} jointAngles - Array of 6 joint angles.
     */
    function updateRobotPose(jointAngles) {
        try {
            const { transforms, T0_TCP } = forwardKinematics(jointAngles);

            // Update positions and orientations of each joint frame (Object3D)
            for (let i = 0; i < jointFrames.length; i++) { // transforms has N+1 elements (T0_0 to T0_N)
                // Apply the calculated absolute transform T0_i to the corresponding frame
                jointFrames[i].matrix.copy(transforms[i]);
                // Tell three.js to update the frame's world matrix based on this local matrix
                // Since we are setting the absolute matrix relative to the scene origin,
                // we need to ensure matrixAutoUpdate is false for these frames OR
                // decompose the matrix into position, quaternion, scale.
                jointFrames[i].matrix.decompose(jointFrames[i].position, jointFrames[i].quaternion, jointFrames[i].scale);
                jointFrames[i].matrixWorldNeedsUpdate = true; // Important
            }

            // Update link meshes (position and orientation between joints)
            for (let i = 0; i < linkMeshes.length; i++) {
                const startPoint = new THREE.Vector3();
                const endPoint = new THREE.Vector3();

                // Get world positions of the start and end joints for this link
                jointFrames[i].getWorldPosition(startPoint); // Link i connects frame i
                jointFrames[i + 1].getWorldPosition(endPoint); // to frame i+1

                const linkVector = new THREE.Vector3().subVectors(endPoint, startPoint);
                const linkLength = linkVector.length();
                const linkMidpoint = new THREE.Vector3().addVectors(startPoint, endPoint).multiplyScalar(0.5);

                const linkMesh = linkMeshes[i];
                linkMesh.scale.set(1, linkLength, 1); // Scale cylinder height
                linkMesh.position.copy(linkMidpoint); // Position at midpoint

                // Align cylinder along the linkVector (aligns cylinder's Y-axis)
                linkMesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), linkVector.normalize());

                // Since link meshes are parented to joint frames, their transforms are relative.
                // It's often easier to manage them directly in world space or reparent them to the scene.
                // Let's reparent them to the scene for simpler world space manipulation.
                scene.add(linkMesh); // Add to scene directly
                linkMesh.matrixWorldNeedsUpdate = true;
            }


            // Update TCP mesh position based on T0_TCP
            // The TCP mesh is parented to the last joint frame (flange),
            // so we only need to set its local transform relative to the flange (H_FLANGE_TCP)
            tcpMesh.matrix.copy(H_FLANGE_TCP);
            tcpMesh.matrix.decompose(tcpMesh.position, tcpMesh.quaternion, tcpMesh.scale);
            tcpMesh.matrixWorldNeedsUpdate = true;


        } catch (error) {
            console.error("Error updating robot pose:", error);
        }
    }


    // --- Initialization ---
    function init() {
        // Scene
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x282c34); // Dark background matching theme

        // Camera
        const aspect = container.clientWidth / container.clientHeight;
        camera = new THREE.PerspectiveCamera(50, aspect, 0.1, 100);
        camera.position.set(1, 1, 1.5); // Initial camera position
        camera.lookAt(0, 0.4, 0); // Look towards the robot base area

        // Renderer
        renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        renderer.shadowMap.enabled = true; // Enable shadows
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        container.appendChild(renderer.domElement);

        // Lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(5, 10, 7.5);
        directionalLight.castShadow = true;
        // Configure shadow properties
        directionalLight.shadow.mapSize.width = 1024;
        directionalLight.shadow.mapSize.height = 1024;
        directionalLight.shadow.camera.near = 0.5;
        directionalLight.shadow.camera.far = 50;
        directionalLight.shadow.camera.left = -5;
        directionalLight.shadow.camera.right = 5;
        directionalLight.shadow.camera.top = 5;
        directionalLight.shadow.camera.bottom = -5;
        scene.add(directionalLight);
        // const lightHelper = new THREE.DirectionalLightHelper(directionalLight, 1);
        // scene.add(lightHelper);
        // const shadowHelper = new THREE.CameraHelper(directionalLight.shadow.camera);
        // scene.add(shadowHelper);


        // Grid Helper
        const gridHelper = new THREE.GridHelper(2, 20, 0x888888, 0x444444); // Size, divisions, color lines, color center
        scene.add(gridHelper);

        // Axes Helper (World Origin)
        const axesHelper = new THREE.AxesHelper(0.5); // Length of axes
        scene.add(axesHelper);

        // Orbit Controls
        if (typeof THREE.OrbitControls !== 'undefined') {
            controls = new THREE.OrbitControls(camera, renderer.domElement);
            controls.target.set(0, 0.4, 0); // Set control target slightly above base
            controls.enableDamping = true; // Smooth camera movement
            controls.dampingFactor = 0.1;
            controls.screenSpacePanning = false; // Keep panning relative to ground plane
            controls.minDistance = 0.5;
            controls.maxDistance = 10;
            controls.maxPolarAngle = Math.PI / 1.9; // Prevent looking from below the grid
            controls.update();
        } else {
            console.warn("OrbitControls not found. Camera interaction will be limited.");
        }


        // Create Robot
        createRobotBase();
        createRobotGeometry(); // Creates joints, links, TCP

        // Handle Window Resize
        window.addEventListener('resize', onWindowResize, false);

        // Start Animation Loop
        animate();
    }

    // --- Animation Loop ---
    function animate() {
        requestAnimationFrame(animate);
        if (controls && controls.enabled) {
            controls.update(); // Required if enableDamping is true
        }
        renderer.render(scene, camera);
    }

    // --- Event Handlers ---
    function onWindowResize() {
        if (!container || !renderer || !camera) return;
        camera.aspect = container.clientWidth / container.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(container.clientWidth, container.clientHeight);
    }

    // Slider and Button Event Listeners
    const sliders = document.querySelectorAll('.joint-slider');
    const valueSpans = {};
    const initialSliderValues = {}; // Store initial values for reset

    sliders.forEach((slider, index) => {
        const jointIndex = index + 1;
        const spanId = `j${jointIndex}-value`;
        valueSpans[jointIndex] = document.getElementById(spanId);
        initialSliderValues[jointIndex] = parseFloat(slider.value); // Store initial value

        // Initial display update
        if (valueSpans[jointIndex]) {
            valueSpans[jointIndex].textContent = parseFloat(slider.value).toFixed(2);
        }

        // Add input event listener
        slider.addEventListener('input', () => {
            const currentJointAngles = Array.from(sliders).map(s => parseFloat(s.value));
            if (valueSpans[jointIndex]) {
                valueSpans[jointIndex].textContent = parseFloat(slider.value).toFixed(2);
            }
            updateRobotPose(currentJointAngles);
        });
    });

    // Reset View Button
    const resetViewBtn = document.getElementById('reset-view-btn');
    if (resetViewBtn && controls) {
        resetViewBtn.addEventListener('click', () => {
            controls.reset(); // Resets camera to saved state
            // Or manually set position/target if needed:
            // camera.position.set(1, 1, 1.5);
            // controls.target.set(0, 0.4, 0);
            controls.update();
        });
    }

    // Reset Pose Button
    const resetPoseBtn = document.getElementById('reset-pose-btn');
    if (resetPoseBtn) {
        resetPoseBtn.addEventListener('click', () => {
            sliders.forEach((slider, index) => {
                const jointIndex = index + 1;
                slider.value = initialSliderValues[jointIndex]; // Reset slider value
                if (valueSpans[jointIndex]) { // Update display
                    valueSpans[jointIndex].textContent = parseFloat(slider.value).toFixed(2);
                }
            });
            // Trigger pose update with initial angles
            updateRobotPose(initialJointAngles);
        });
    }


    // --- Start Everything ---
    init();

}); // End DOMContentLoaded
