// Interactive Kinematics JavaScript for UR5e Robot
document.addEventListener('DOMContentLoaded', function() {
    // DH parameters for UR5e
    const DH_PARAMS_UR5E = [
        {a: 0.0,     alpha: Math.PI/2,  d: 0.1625, theta_offset: 0.0},
        {a: -0.425,  alpha: 0.0,        d: 0.0,    theta_offset: 0.0},
        {a: -0.3922, alpha: 0.0,        d: 0.0,    theta_offset: 0.0},
        {a: 0.0,     alpha: Math.PI/2,  d: 0.1333, theta_offset: 0.0},
        {a: 0.0,     alpha: -Math.PI/2, d: 0.0997, theta_offset: 0.0},
        {a: 0.0,     alpha: 0.0,        d: 0.0996, theta_offset: 0.0}
    ];

    // TCP offset
    const TCP_Z_OFFSET = 0.1565;

    // Constants for calculations
    const TOL_ZERO = 1e-9;
    const TOL_SINGULARITY = 1e-7;
    const TOL_COMPARE = 1e-6;
    const TOL_GEOM = 1e-6;

    // Extract commonly used values from DH parameters
    const d1 = DH_PARAMS_UR5E[0].d;
    const a2 = DH_PARAMS_UR5E[1].a;
    const a3 = DH_PARAMS_UR5E[2].a;
    const d4 = DH_PARAMS_UR5E[3].d;
    const d5 = DH_PARAMS_UR5E[4].d;
    const d6 = DH_PARAMS_UR5E[5].d;
    const len_a2 = Math.abs(a2);
    const len_a3 = Math.abs(a3);

    // H_FLANGE_TCP transformation matrix
    const H_FLANGE_TCP = [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, TCP_Z_OFFSET],
        [0, 0, 0, 1]
    ];

    // Inverse of H_FLANGE_TCP
    const INV_H_FLANGE_TCP = [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, -TCP_Z_OFFSET],
        [0, 0, 0, 1]
    ];

    // Reference to DOM elements
    const sliders = {
        joint1: document.getElementById('joint1'),
        joint2: document.getElementById('joint2'),
        joint3: document.getElementById('joint3'),
        joint4: document.getElementById('joint4'),
        joint5: document.getElementById('joint5'),
        joint6: document.getElementById('joint6')
    };

    const sliderValues = {
        joint1: document.getElementById('joint1-value'),
        joint2: document.getElementById('joint2-value'),
        joint3: document.getElementById('joint3-value'),
        joint4: document.getElementById('joint4-value'),
        joint5: document.getElementById('joint5-value'),
        joint6: document.getElementById('joint6-value')
    };

    const tcpPoseElement = document.getElementById('tcp-pose-matrix');
    const ikSolutionsCountElement = document.getElementById('ik-solutions-count');
    const ikSolutionsListElement = document.getElementById('ik-solutions-list');

    // Current joint angle values
    let currentJointAngles = [
        parseFloat(sliders.joint1.value),
        parseFloat(sliders.joint2.value),
        parseFloat(sliders.joint3.value),
        parseFloat(sliders.joint4.value),
        parseFloat(sliders.joint5.value),
        parseFloat(sliders.joint6.value)
    ];

    // Current TCP pose
    let currentTcpPose = null;
    
    // IK solutions
    let ikSolutions = [];
    
    // Active solution index
    let activeSolutionIndex = -1;
    
    // Colors for different robot configurations
    const COLORS = {
        mainRobot: 0x2196f3,   // Blue
        ikSolutions: [
            0x4caf50,          // Green
            0xff9800,          // Orange
            0xe91e63,          // Pink
            0x9c27b0,          // Purple
            0x00bcd4,          // Cyan
            0xffeb3b,          // Yellow
            0x795548,          // Brown
            0x607d8b           // Blue Grey
        ]
    };

    // THREE.js variables
    let scene, camera, renderer, controls;
    let mainRobot = null;
    let ikRobots = [];
    
    // Initialize the 3D scene
    function initVisualization() {
        const container = document.getElementById('robot-visualization');
        
        // Create scene
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x1a1a2e);
        
        // Create camera
        camera = new THREE.PerspectiveCamera(
            45, 
            container.clientWidth / container.clientHeight, 
            0.1, 
            1000
        );
        camera.position.set(1.5, 1, 1.5);
        
        // Create renderer
        renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        container.appendChild(renderer.domElement);
        
        // Add orbit controls
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.1;
        
        // Add grid
        const gridHelper = new THREE.GridHelper(2, 20, 0x222222, 0x333333);
        scene.add(gridHelper);
        
        // Add axes
        const axesHelper = new THREE.AxesHelper(0.3);
        scene.add(axesHelper);
        
        // Add ambient light
        scene.add(new THREE.AmbientLight(0xffffff, 0.6));
        
        // Add directional light
        const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
        dirLight.position.set(5, 10, 5);
        scene.add(dirLight);
        
        // Create main robot
        mainRobot = createRobotModel(COLORS.mainRobot);
        scene.add(mainRobot.group);
        
        // Handle window resize
        window.addEventListener('resize', onWindowResize);
        
        // Start animation loop
        animate();
        
        // Initial update
        updateRobotVisualization();
    }
    
    // Create a robot arm model
    function createRobotModel(color) {
        const group = new THREE.Group();
        const links = [];
        const joints = [];
        
        // Base
        const baseGeometry = new THREE.CylinderGeometry(0.1, 0.1, 0.05, 32);
        const baseMaterial = new THREE.MeshStandardMaterial({ color: 0x333333 });
        const base = new THREE.Mesh(baseGeometry, baseMaterial);
        base.position.y = -0.025;
        group.add(base);
        
        // Create materials
        const jointMaterial = new THREE.MeshStandardMaterial({ color: 0x555555 });
        const linkMaterial = new THREE.MeshStandardMaterial({ color: color });
        
        // Create empty objects to serve as joints and links
        for (let i = 0; i < 7; i++) {
            const joint = new THREE.Group();
            joints.push(joint);
            
            if (i > 0) {
                // Add visual representation for joints
                const jointGeometry = new THREE.SphereGeometry(0.04, 32, 32);
                const jointMesh = new THREE.Mesh(jointGeometry, jointMaterial);
                joint.add(jointMesh);
            }
            
            if (i < 6) {
                // Create links between joints
                const link = new THREE.Group();
                links.push(link);
                joint.add(link);
            }
        }
        
        // Assemble the kinematic chain
        group.add(joints[0]);
        for (let i = 1; i < joints.length; i++) {
            joints[i-1].add(joints[i]);
        }
        
        // Add TCP visualization
        const tcpGroup = new THREE.Group();
        const tcpGeometry = new THREE.SphereGeometry(0.02, 16, 16);
        const tcpMaterial = new THREE.MeshStandardMaterial({ color: 0xff0000 });
        const tcpMesh = new THREE.Mesh(tcpGeometry, tcpMaterial);
        tcpGroup.add(tcpMesh);
        
        // Add TCP axes
        const tcpAxesHelper = new THREE.AxesHelper(0.1);
        tcpGroup.add(tcpAxesHelper);
        
        joints[6].add(tcpGroup);
        
        return {
            group,
            joints,
            links,
            tcpGroup
        };
    }
    
    // Handle window resize
    function onWindowResize() {
        const container = document.getElementById('robot-visualization');
        camera.aspect = container.clientWidth / container.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(container.clientWidth, container.clientHeight);
    }
    
    // Animation loop
    function animate() {
        requestAnimationFrame(animate);
        controls.update();
        renderer.render(scene, camera);
    }
    
    // Create transformation matrix from DH parameters
    function dhMatrix(a, alpha, d, theta) {
        const cosTheta = Math.cos(theta);
        const sinTheta = Math.sin(theta);
        const cosAlpha = Math.cos(alpha);
        const sinAlpha = Math.sin(alpha);
        
        return [
            [cosTheta, -sinTheta * cosAlpha,  sinTheta * sinAlpha, a * cosTheta],
            [sinTheta,  cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta],
            [0,         sinAlpha,             cosAlpha,           d],
            [0,         0,                    0,                  1]
        ];
    }
    
    // Matrix multiplication
    function multiplyMatrices(a, b) {
        const result = [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ];
        
        for (let i = 0; i < 4; i++) {
            for (let j = 0; j < 4; j++) {
                for (let k = 0; k < 4; k++) {
                    result[i][j] += a[i][k] * b[k][j];
                }
            }
        }
        
        return result;
    }
    
    // Forward kinematics calculation
    function forwardKinematics(jointAngles) {
        if (jointAngles.length !== DH_PARAMS_UR5E.length) {
            throw new Error(`Angle count mismatch: Expected ${DH_PARAMS_UR5E.length}, got ${jointAngles.length}`);
        }
        
        const transforms = [identityMatrix()];
        let prevT = transforms[0];
        
        for (let i = 0; i < DH_PARAMS_UR5E.length; i++) {
            const p = DH_PARAMS_UR5E[i];
            const Ti = dhMatrix(p.a, p.alpha, p.d, jointAngles[i] + p.theta_offset);
            const currT = multiplyMatrices(prevT, Ti);
            transforms.push(currT);
            prevT = currT;
        }
        
        const T0_flange = transforms[transforms.length - 1];
        const T0_TCP = multiplyMatrices(T0_flange, H_FLANGE_TCP);
        transforms.push(T0_TCP);
        
        const points = transforms.map(T => [T[0][3], T[1][3], T[2][3]]);
        
        return {
            points,
            T0_TCP,
            T0_flange
        };
    }
    
    // Create identity matrix
    function identityMatrix() {
        return [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ];
    }
    
    // Matrix transpose (for rotation matrix)
    function transposeMatrix(m) {
        return [
            [m[0][0], m[1][0], m[2][0], m[3][0]],
            [m[0][1], m[1][1], m[2][1], m[3][1]],
            [m[0][2], m[1][2], m[2][2], m[3][2]],
            [m[0][3], m[1][3], m[2][3], m[3][3]]
        ];
    }
    
    // Matrix inverse (simplified for transformation matrices)
    function invertTransformMatrix(m) {
        // Extract rotation matrix (3x3)
        const r = [
            [m[0][0], m[0][1], m[0][2]],
            [m[1][0], m[1][1], m[1][2]],
            [m[2][0], m[2][1], m[2][2]]
        ];
        
        // Extract translation vector
        const t = [m[0][3], m[1][3], m[2][3]];
        
        // Transpose of rotation matrix is its inverse
        const rT = [
            [r[0][0], r[1][0], r[2][0]],
            [r[0][1], r[1][1], r[2][1]],
            [r[0][2], r[1][2], r[2][2]]
        ];
        
        // Calculate -R^T * t
        const negRtT = [
            -(rT[0][0] * t[0] + rT[0][1] * t[1] + rT[0][2] * t[2]),
            -(rT[1][0] * t[0] + rT[1][1] * t[1] + rT[1][2] * t[2]),
            -(rT[2][0] * t[0] + rT[2][1] * t[1] + rT[2][2] * t[2])
        ];
        
        // Build inverse transformation matrix
        return [
            [rT[0][0], rT[0][1], rT[0][2], negRtT[0]],
            [rT[1][0], rT[1][1], rT[1][2], negRtT[1]],
            [rT[2][0], rT[2][1], rT[2][2], negRtT[2]],
            [0, 0, 0, 1]
        ];
    }
    
    // Inverse kinematics calculation
    function inverseKinematics(T_desired_TCP) {
        const solutions = [];
        
        // Get T_flange by removing TCP transform
        const T_flange = multiplyMatrices(T_desired_TCP, INV_H_FLANGE_TCP);
        
        // Extract position and rotation from T_flange
        const P60 = [T_flange[0][3], T_flange[1][3], T_flange[2][3]];
        
        // First extract P50 (wrist center) from P60
        const R06_col3 = [T_flange[0][2], T_flange[1][2], T_flange[2][2]];
        const P50 = [
            P60[0] - d6 * R06_col3[0],
            P60[1] - d6 * R06_col3[1],
            P60[2] - d6 * R06_col3[2]
        ];
        
        const P50x = P50[0];
        const P50y = P50[1];
        const P50z = P50[2];
        
        // Calculate theta1 (joint 1)
        const A = P50y;
        const B = P50x;
        
        const dist_sq_xy = B * B + A * A;
        
        // Check if P50 is out of reach
        if (dist_sq_xy < d4 * d4 - TOL_GEOM) {
            return [];
        }
        
        // Check for singularity
        if (Math.abs(B) < TOL_SINGULARITY && Math.abs(A) < TOL_SINGULARITY) {
            return [];
        }
        
        // Calculate theta1 solutions
        const sqrt_arg_t1 = Math.max(0, dist_sq_xy - d4 * d4);
        const sqrt_val_t1 = Math.sqrt(sqrt_arg_t1);
        
        const term1_t1 = Math.atan2(B, -A);
        const term2_t1 = Math.atan2(sqrt_val_t1, d4);
        
        const theta1_sol = [term1_t1 + term2_t1, term1_t1 - term2_t1];
        
        // For each theta1 solution, continue to solve for the rest
        for (let t1_idx = 0; t1_idx < theta1_sol.length; t1_idx++) {
            const t1 = theta1_sol[t1_idx];
            const s1 = Math.sin(t1);
            const c1 = Math.cos(t1);
            
            // Extract values from rotation matrix
            const R06 = [
                [T_flange[0][0], T_flange[0][1], T_flange[0][2]],
                [T_flange[1][0], T_flange[1][1], T_flange[1][2]],
                [T_flange[2][0], T_flange[2][1], T_flange[2][2]]
            ];
            
            const r11d = R06[0][0], r12d = R06[0][1], r13d = R06[0][2];
            const r21d = R06[1][0], r22d = R06[1][1], r23d = R06[1][2];
            const r31d = R06[2][0], r32d = R06[2][1], r33d = R06[2][2];
            
            // Calculate M, D, E for theta5
            const M_val = s1 * r13d - c1 * r23d;
            const M = Math.min(Math.max(M_val, -1.0), 1.0);  // Clip to [-1, 1]
            
            const D = c1 * r22d - s1 * r12d;
            const E = s1 * r11d - c1 * r21d;
            
            const sqrt_arg_ED = E * E + D * D;
            
            // Verify valid configuration
            if (Math.abs(M * M + sqrt_arg_ED - 1.0) > TOL_COMPARE) {
                continue;
            }
            
            const sqrt_val_ED = Math.sqrt(Math.max(0, sqrt_arg_ED));
            
            if (Math.abs(sqrt_val_ED) < TOL_SINGULARITY && Math.abs(M) < TOL_SINGULARITY) {
                continue;
            }
            
            // Calculate theta5 solutions
            const t5_sol = [Math.atan2(sqrt_val_ED, M), Math.atan2(-sqrt_val_ED, M)];
            
            for (let t5_idx = 0; t5_idx < t5_sol.length; t5_idx++) {
                const t5 = t5_sol[t5_idx];
                const s5 = Math.sin(t5);
                const c5 = Math.cos(t5);
                
                // Calculate theta6
                let t6 = 0.0;
                const is_singular_s5 = Math.abs(s5) < TOL_SINGULARITY;
                const is_singular_c5 = Math.abs(c5) < TOL_SINGULARITY;
                
                if (is_singular_s5 || is_singular_c5) {
                    t6 = 0.0;  // Arbitrary value for singularity case
                } else {
                    if (Math.abs(D) < TOL_SINGULARITY && Math.abs(E) < TOL_SINGULARITY) {
                        continue;
                    }
                    
                    t6 = s5 > 0 ? Math.atan2(D, E) : Math.atan2(-D, -E);
                }
                
                const s6 = Math.sin(t6);
                const c6 = Math.cos(t6);
                
                // Calculate theta234 (combined angle for joints 2, 3, 4)
                const F = c5 * c6;
                const C_val = c1 * r11d + s1 * r21d;
                
                const atan_y_234 = r31d * F - s6 * C_val;
                const atan_x_234 = F * C_val + s6 * r31d;
                
                if (Math.abs(atan_y_234) < TOL_SINGULARITY && Math.abs(atan_x_234) < TOL_SINGULARITY) {
                    continue;
                }
                
                const t234 = Math.atan2(atan_y_234, atan_x_234);
                const s234 = Math.sin(t234);
                const c234 = Math.cos(t234);
                
                // Calculate K constants for triangular solution
                const KC = c1 * P50x + s1 * P50y - s234 * d5 + c234 * s5 * d6;
                const KS = P50z - d1 + c234 * d5 + s234 * s5 * d6;
                
                const dist_sq_13 = KC * KC + KS * KS;
                const denom_t3 = 2 * len_a2 * len_a3;
                
                if (Math.abs(denom_t3) < TOL_ZERO) {
                    continue;
                }
                
                // Calculate theta3 using cosine law
                const cos_t3_arg = (dist_sq_13 - len_a2 * len_a2 - len_a3 * len_a3) / denom_t3;
                
                if (Math.abs(cos_t3_arg) > 1.0 + TOL_GEOM) {
                    continue;
                }
                
                const cos_t3_arg_clamped = Math.min(Math.max(cos_t3_arg, -1.0), 1.0);
                const sqrt_arg_t3 = 1.0 - cos_t3_arg_clamped * cos_t3_arg_clamped;
                const sqrt_val_t3 = Math.sqrt(Math.max(0, sqrt_arg_t3));
                
                // Two possible solutions for theta3
                const t3_sol = [
                    Math.atan2(sqrt_val_t3, cos_t3_arg_clamped),
                    Math.atan2(-sqrt_val_t3, cos_t3_arg_clamped)
                ];
                
                for (let t3_idx = 0; t3_idx < t3_sol.length; t3_idx++) {
                    const t3 = t3_sol[t3_idx];
                    const s3 = Math.sin(t3);
                    const c3 = cos_t3_arg_clamped;
                    
                    if (Math.abs(KS) < TOL_SINGULARITY && Math.abs(KC) < TOL_SINGULARITY) {
                        continue;
                    }
                    
                    // Calculate theta2
                    const term1_t2 = Math.atan2(KS, KC);
                    const term2_y_t2 = a3 * s3;
                    const term2_x_t2 = a2 + a3 * c3;
                    
                    if (Math.abs(term2_y_t2) < TOL_SINGULARITY && Math.abs(term2_x_t2) < TOL_SINGULARITY) {
                        continue;
                    }
                    
                    const term2_t2 = Math.atan2(term2_y_t2, term2_x_t2);
                    const t2 = term1_t2 - term2_t2;
                    
                    // Calculate theta4
                    const t4 = t234 - t2 - t3;
                    
                    // Normalize angles to [-π, π]
                    const sol_angles_raw = [t1, t2, t3, t4, t5, t6];
                    const sol_angles_normalized = sol_angles_raw.map(a => ((a + Math.PI) % (2 * Math.PI)) - Math.PI);
                    
                    // Verify solution using forward kinematics
                    try {
                        const { T0_flange: T0_flange_check } = forwardKinematics(sol_angles_normalized);
                        const is_valid = isMatrixClose(T0_flange_check, T_flange, TOL_COMPARE);
                        
                        if (is_valid) {
                            solutions.push({
                                angles: sol_angles_normalized,
                                indices: [t1_idx, t5_idx, t3_idx]
                            });
                        }
                    } catch (e) {
                        console.error("Error validating IK solution:", e);
                        continue;
                    }
                }
            }
        }
        
        return solutions;
    }
    
    // Check if two matrices are approximately equal
    function isMatrixClose(m1, m2, tolerance) {
        for (let i = 0; i < 4; i++) {
            for (let j = 0; j < 4; j++) {
                if (Math.abs(m1[i][j] - m2[i][j]) > tolerance) {
                    return false;
                }
            }
        }
        return true;
    }
    
    // Format number to 2 decimal places
    function formatNumber(num) {
        return num.toFixed(2);
    }
    
    // Update the robot visualization based on joint angles
    function updateRobotVisualization() {
        try {
            // Calculate forward kinematics
            const { points, T0_TCP } = forwardKinematics(currentJointAngles);
            currentTcpPose = T0_TCP;
            
            // Update main robot visualization
            updateRobotJoints(mainRobot, currentJointAngles, points);
            
            // Update slider value displays
            sliderValues.joint1.textContent = formatNumber(currentJointAngles[0]);
            sliderValues.joint2.textContent = formatNumber(currentJointAngles[1]);
            sliderValues.joint3.textContent = formatNumber(currentJointAngles[2]);
            sliderValues.joint4.textContent = formatNumber(currentJointAngles[3]);
            sliderValues.joint5.textContent = formatNumber(currentJointAngles[4]);
            sliderValues.joint6.textContent = formatNumber(currentJointAngles[5]);
            
            // Update TCP pose matrix display
            updateTcpPoseDisplay(T0_TCP);
            
            // Calculate inverse kinematics solutions
            calculateIkSolutions(T0_TCP);
            
        } catch (error) {
            console.error("Error updating visualization:", error);
        }
    }
    
    // Update the TCP pose display
    function updateTcpPoseDisplay(T0_TCP) {
        const rows = tcpPoseElement.querySelectorAll('.matrix-row');
        
        for (let i = 0; i < 4; i++) {
            const spans = rows[i].querySelectorAll('span');
            for (let j = 0; j < 4; j++) {
                spans[j].textContent = formatNumber(T0_TCP[i][j]);
            }
        }
    }
    
    // Update robot joint positions
    function updateRobotJoints(robot, jointAngles, points) {
        const rotations = [
            jointAngles[0],                        // Joint 1: Rotate around Y
            jointAngles[1] + Math.PI / 2,          // Joint 2: Rotate around Z
            jointAngles[2],                        // Joint 3: Rotate around Z
            jointAngles[3] + Math.PI / 2,          // Joint 4: Rotate around Y
            jointAngles[4],                        // Joint 5: Rotate around Z
            jointAngles[5],                        // Joint 6: Rotate around Y
        ];
        
        // Reset all rotations
        robot.joints.forEach(joint => {
            joint.rotation.set(0, 0, 0);
            joint.position.set(0, 0, 0);
        });
        
        // Set rotations for each joint
        robot.joints[0].position.y = 0;
        robot.joints[0].rotation.y = rotations[0];
        
        robot.joints[1].position.y = DH_PARAMS_UR5E[0].d;
        robot.joints[1].rotation.z = rotations[1];
        
        robot.joints[2].position.z = 0;
        robot.joints[2].position.x = DH_PARAMS_UR5E[1].a;
        robot.joints[2].rotation.z = rotations[2];
        
        robot.joints[3].position.z = 0;
        robot.joints[3].position.x = DH_PARAMS_UR5E[2].a;
        robot.joints[3].rotation.y = rotations[3];
        
        robot.joints[4].position.y = DH_PARAMS_UR5E[3].d;
        robot.joints[4].rotation.z = rotations[4];
        
        robot.joints[5].position.y = DH_PARAMS_UR5E[4].d;
        robot.joints[5].rotation.y = rotations[5];
        
        robot.joints[6].position.y = DH_PARAMS_UR5E[5].d;
        
        // Update the visual links between joints
        for (let i = 0; i < robot.links.length; i++) {
            const startPoint = points[i];
            const endPoint = points[i + 1];
            
            const length = Math.sqrt(
                Math.pow(endPoint[0] - startPoint[0], 2) +
                Math.pow(endPoint[1] - startPoint[1], 2) +
                Math.pow(endPoint[2] - startPoint[2], 2)
            );
            
            // Update or create link geometry
            if (!robot.links[i].children.length) {
                const linkGeometry = new THREE.CylinderGeometry(0.02, 0.02, length, 8);
                linkGeometry.translate(0, length / 2, 0);
                linkGeometry.rotateX(Math.PI / 2);
                
                const linkMaterial = new THREE.MeshStandardMaterial({ color: robot === mainRobot ? COLORS.mainRobot : COLORS.ikSolutions[ikRobots.indexOf(robot) % COLORS.ikSolutions.length] });
                const linkMesh = new THREE.Mesh(linkGeometry, linkMaterial);
                
                robot.links[i].add(linkMesh);
            } else {
                const linkMesh = robot.links[i].children[0];
                
                // Resize and reorient the link
                linkMesh.scale.y = length;
                
                // Reposition the mesh to connect the joints
                linkMesh.position.set(0, 0, 0);
            }
            
            // Set the look-at target dynamically to orient the link
            const direction = new THREE.Vector3(
                endPoint[0] - startPoint[0],
                endPoint[1] - startPoint[1],
                endPoint[2] - startPoint[2]
            );
            
            if (direction.length() > 0.001) {
                robot.links[i].lookAt(direction.add(robot.links[i].getWorldPosition(new THREE.Vector3())));
            }
        }
    }
    
    // Calculate IK solutions and update visualization
    function calculateIkSolutions(T0_TCP) {
        // Clear old IK robots
        ikRobots.forEach(robot => {
            scene.remove(robot.group);
        });
        ikRobots = [];
        
        // Calculate new IK solutions
        ikSolutions = inverseKinematics(T0_TCP);
        
        // Update IK solutions count
        ikSolutionsCountElement.textContent = `Found ${ikSolutions.length} solutions`;
        
        // Clear existing IK solutions list
        ikSolutionsListElement.innerHTML = '';
        
        // Create new robots for each IK solution
        ikSolutions.forEach((solution, index) => {
            // Display solution in the list
            const solutionItem = document.createElement('div');
            solutionItem.className = 'solution-item';
            solutionItem.setAttribute('data-index', index);
            
            // Format solution text
            const solutionAngles = document.createElement('div');
            solutionAngles.className = 'solution-angles';
            
            solution.angles.forEach((angle, i) => {
                const angleSpan = document.createElement('div');
                angleSpan.className = 'solution-angle';
                angleSpan.textContent = `J${i+1}: ${formatNumber(angle)}`;
                solutionAngles.appendChild(angleSpan);
            });
            
            solutionItem.appendChild(solutionAngles);
            ikSolutionsListElement.appendChild(solutionItem);
            
            // Create 3D visualization for each solution
            const ikRobot = createRobotModel(COLORS.ikSolutions[index % COLORS.ikSolutions.length]);
            const { points } = forwardKinematics(solution.angles);
            
            // Update joints and make semi-transparent
            updateRobotJoints(ikRobot, solution.angles, points);
            ikRobot.group.traverse(obj => {
                if (obj.isMesh) {
                    obj.material.transparent = true;
                    obj.material.opacity = 0.3;
                }
            });
            
            // Make invisible initially if not the active solution
            ikRobot.group.visible = false;
            
            scene.add(ikRobot.group);
            ikRobots.push(ikRobot);
            
            // Add click handler for solution selection
            solutionItem.addEventListener('click', () => {
                setActiveSolution(index);
            });
        });
        
        // Set the first solution as active if available
        if (ikSolutions.length > 0) {
            setActiveSolution(0);
        } else {
            activeSolutionIndex = -1;
        }
    }
    
    // Set the active IK solution for display
    function setActiveSolution(index) {
        // Update active solution index
        activeSolutionIndex = index;
        
        // Update UI
        const solutionItems = ikSolutionsListElement.querySelectorAll('.solution-item');
        solutionItems.forEach(item => {
            item.classList.remove('active');
            const itemIndex = parseInt(item.getAttribute('data-index'));
            if (itemIndex === index) {
                item.classList.add('active');
            }
        });
        
        // Show only the active solution in 3D
        ikRobots.forEach((robot, i) => {
            robot.group.visible = (i === index);
        });
    }
    
    // Handle slider input
    function handleSliderInput() {
        currentJointAngles = [
            parseFloat(sliders.joint1.value),
            parseFloat(sliders.joint2.value),
            parseFloat(sliders.joint3.value),
            parseFloat(sliders.joint4.value),
            parseFloat(sliders.joint5.value),
            parseFloat(sliders.joint6.value)
        ];
        
        updateRobotVisualization();
    }
    
    // Apply IK solution to the sliders
    function applyIkSolution(index) {
        if (index >= 0 && index < ikSolutions.length) {
            const solution = ikSolutions[index];
            
            // Update sliders
            sliders.joint1.value = solution.angles[0];
            sliders.joint2.value = solution.angles[1];
            sliders.joint3.value = solution.angles[2];
            sliders.joint4.value = solution.angles[3];
            sliders.joint5.value = solution.angles[4];
            sliders.joint6.value = solution.angles[5];
            
            // Update current joint angles
            currentJointAngles = [...solution.angles];
            
            // Update visualization
            updateRobotVisualization();
        }
    }
    
    // Attach slider event listeners
    for (const key in sliders) {
        sliders[key].addEventListener('input', handleSliderInput);
    }
    
    // Initialize the visualization
    initVisualization();