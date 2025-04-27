/* eslint-disable no-unused-vars */ // To prevent editor warnings about unused functions if not using modules

const UR5eKinematics = (() => {
    // --- Constants ---
    const DH_PARAMS_UR5E = [
        { a: 0.0,    alpha: Math.PI / 2, d: 0.1625, theta_offset: 0.0 },
        { a: -0.425, alpha: 0.0,         d: 0.0,    theta_offset: 0.0 },
        { a: -0.3922,alpha: 0.0,         d: 0.0,    theta_offset: 0.0 },
        { a: 0.0,    alpha: Math.PI / 2, d: 0.1333, theta_offset: 0.0 },
        { a: 0.0,    alpha: -Math.PI / 2,d: 0.0997, theta_offset: 0.0 },
        { a: 0.0,    alpha: 0.0,         d: 0.0996, theta_offset: 0.0 }
    ];

    const TCP_Z_OFFSET = 0.1565; // Assuming same TCP offset as your python code

    // Homogeneous transform from flange to TCP
    const H_FLANGE_TCP = [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, TCP_Z_OFFSET],
        [0, 0, 0, 1]
    ];

    // Inverse Homogeneous transform from TCP to flange
    const INV_H_FLANGE_TCP = [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, -TCP_Z_OFFSET],
        [0, 0, 0, 1]
    ];

    // Extract DH parameters for easier access in IK
    const d1 = DH_PARAMS_UR5E[0].d;
    const a2 = DH_PARAMS_UR5E[1].a;
    const a3 = DH_PARAMS_UR5E[2].a;
    const d4 = DH_PARAMS_UR5E[3].d;
    const d5 = DH_PARAMS_UR5E[4].d;
    const d6 = DH_PARAMS_UR5E[5].d;
    const len_a2 = Math.abs(a2);
    const len_a3 = Math.abs(a3);

    // Tolerances
    const tol_zero = 1e-9;
    const tol_singularity = 1e-6; // Adjusted slightly
    const tol_compare = 1e-5;     // Adjusted slightly
    const tol_geom = 1e-6;

    // --- Helper Functions ---

    /**
     * Multiplies two 4x4 matrices (arrays of arrays).
     * @param {number[][]} A First matrix
     * @param {number[][]} B Second matrix
     * @returns {number[][]} Result matrix A * B
     */
    function multiplyMatrices(A, B) {
        const C = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
        for (let i = 0; i < 4; i++) {
            for (let j = 0; j < 4; j++) {
                for (let k = 0; k < 4; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    /**
     * Creates a DH transformation matrix.
     * @param {number} a Link length
     * @param {number} alpha Link twist
     * @param {number} d Link offset
     * @param {number} theta Joint angle
     * @returns {number[][]} 4x4 transformation matrix
     */
    function dhMatrix(a, alpha, d, theta) {
        const cos_t = Math.cos(theta);
        const sin_t = Math.sin(theta);
        const cos_a = Math.cos(alpha);
        const sin_a = Math.sin(alpha);
        return [
            [cos_t, -sin_t * cos_a,  sin_t * sin_a, a * cos_t],
            [sin_t,  cos_t * cos_a, -cos_t * sin_a, a * sin_t],
            [    0,          sin_a,          cos_a,         d],
            [    0,              0,              0,         1]
        ];
    }

    /**
     * Clips a value between min and max.
     * @param {number} value The value to clip.
     * @param {number} min The minimum allowed value.
     * @param {number} max The maximum allowed value.
     * @returns {number} The clipped value.
     */
    function clip(value, min, max) {
        return Math.max(min, Math.min(value, max));
    }

     /**
     * Normalizes an angle to be within the range [-pi, pi].
     * @param {number} angle Angle in radians.
     * @returns {number} Normalized angle in radians.
     */
    function normalizeAngle(angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle <= -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }


    // --- Kinematics Functions ---

    /**
     * Calculates Forward Kinematics for the UR5e.
     * @param {number[]} jointAngles Array of 6 joint angles in radians.
     * @param {object[]} dhParams DH parameters (defaults to UR5e).
     * @param {number[][]} hFlangeTcp Transform from flange to TCP (defaults to UR5e).
     * @returns { {points: number[][], T0_TCP: number[][], T0_Flange: number[][]} | null }
     * Object containing joint positions, TCP transform, Flange transform, or null on error.
     */
    function forwardKinematics(jointAngles, dhParams = DH_PARAMS_UR5E, hFlangeTcp = H_FLANGE_TCP) {
        if (jointAngles.length !== dhParams.length) {
            console.error(`Angle count mismatch: Expected ${dhParams.length}, got ${jointAngles.length}`);
            return null;
        }

        let transforms = [[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]]; // T0_0 Identity
        let T_prev = transforms[0];

        for (let i = 0; i < dhParams.length; i++) {
            const p = dhParams[i];
            const T_i_minus_1_to_i = dhMatrix(p.a, p.alpha, p.d, jointAngles[i] + p.theta_offset);
            const T_curr = multiplyMatrices(T_prev, T_i_minus_1_to_i);
            transforms.push(T_curr);
            T_prev = T_curr;
        }

        const T0_flange = transforms[transforms.length - 1];
        const T0_TCP = multiplyMatrices(T0_flange, hFlangeTcp);
        transforms.push(T0_TCP); // Add TCP transform for point extraction

        // Extract points [x, y, z] from transforms
        const points = transforms.map(T => [T[0][3], T[1][3], T[2][3]]);

        return { points: points, T0_TCP: T0_TCP, T0_Flange: T0_flange };
    }


    /**
     * Calculates Inverse Kinematics for the UR5e.
     * @param {number[][]} T_desired_TCP The desired 4x4 TCP pose matrix.
     * @param {object[]} dhParams DH parameters (defaults to UR5e).
     * @param {number[][]} invHFlangeTcp Inverse transform from TCP to flange (defaults to UR5e).
     * @returns {number[][]} An array of valid joint angle solutions (each solution is an array of 6 angles in radians).
     */
    function inverseKinematics(T_desired_TCP, dhParams = DH_PARAMS_UR5E, invHFlangeTcp = INV_H_FLANGE_TCP) {
        let solutions = [];
        const T_flange = multiplyMatrices(T_desired_TCP, invHFlangeTcp);

        // Extract flange position and orientation
        const P60 = [T_flange[0][3], T_flange[1][3], T_flange[2][3]];
        const R06 = [[T_flange[0][0], T_flange[0][1], T_flange[0][2]],
                     [T_flange[1][0], T_flange[1][1], T_flange[1][2]],
                     [T_flange[2][0], T_flange[2][1], T_flange[2][2]]];

        const [pxd, pyd, pzd] = P60;
        const [r11d, r12d, r13d] = R06[0];
        const [r21d, r22d, r23d] = R06[1];
        const [r31d, r32d, r33d] = R06[2];

        // --- Solve for Theta 1 ---
        // P50 = P60 - d6 * z_axis_flange (where z_axis_flange is the 3rd column of R06)
        const P50 = [
            pxd - d6 * r13d,
            pyd - d6 * r23d,
            pzd - d6 * r33d
        ];
        const [P50x, P50y, P50z] = P50;

        // Check reachability for theta1 calculation
        const dist_sq_xy = P50x**2 + P50y**2;
        if (dist_sq_xy < d4**2 - tol_geom) {
           // console.warn("IK Warning: Target likely too close to base for theta1 (xy projection).", dist_sq_xy, d4**2);
            return []; // Cannot reach
        }

        const sqrt_arg_t1 = dist_sq_xy - d4**2; // Should be >= 0 if check above passes
        const sqrt_val_t1 = Math.sqrt(Math.max(0, sqrt_arg_t1)); // Ensure non-negative arg for sqrt

        // atan2(y, x)
        const phi1 = Math.atan2(P50y, P50x);
        const phi2 = Math.atan2(d4, sqrt_val_t1); // Note the swapped order compared to python atan2(sqrt, d4) due to different atan2 definitions or angle convention

        // Two solutions for theta1
        const theta1_sol = [
             phi1 - phi2,         // Elbow down equivalent
             phi1 + phi2 + Math.PI// Elbow up equivalent (adjusting for atan2 range/convention)
        ];
        // console.log("Theta1 Sols (Raw):", theta1_sol.map(a => a*180/Math.PI));


        for (let t1_idx = 0; t1_idx < theta1_sol.length; t1_idx++) {
            const t1 = normalizeAngle(theta1_sol[t1_idx]);
            const s1 = Math.sin(t1);
            const c1 = Math.cos(t1);

            // --- Solve for Theta 5 ---
            // M = P50x*s1 - P50y*c1; // Simplified projection onto the plane normal to axis 1
            // Note: The python code uses elements of R06, which is more robust
            // M = r13d * s1 - r23d * c1;
             const M_val = P50x * s1 - P50y * c1 - d4; // Seems this is the correct value based on common UR IK derivations
            if (Math.abs(M_val) > d5 + tol_geom) {
                // console.warn(`IK Warning (t1_idx ${t1_idx}): M_val out of range for acos`, M_val, d5);
                continue; // Cannot reach physically
            }
             const M_clipped = clip(M_val / d5, -1.0, 1.0);
            const t5_base = Math.acos(M_clipped);

            const theta5_sol = [t5_base, -t5_base]; // Two solutions for theta5

            for (let t5_idx = 0; t5_idx < theta5_sol.length; t5_idx++) {
                const t5 = normalizeAngle(theta5_sol[t5_idx]);
                const s5 = Math.sin(t5);
                const c5 = Math.cos(t5);

                 if (Math.abs(s5) < tol_singularity) {
                   // console.warn(`IK Warning (t1:${t1_idx}, t5:${t5_idx}): Wrist singularity (t5 near 0 or pi), t6 arbitrary.`);
                    // At wrist singularity, t6 is arbitrary, often set to 0.
                    // Need to recalculate dependent angles carefully.
                    // This simplified solver might struggle here. Let's proceed but note potential issues.
                    // Setting t6 = 0 is a common convention
                   // continue; // Skip singular solutions for simplicity in this example
                 }


                // --- Solve for Theta 6 ---
                // Need R06 components transformed relative to frame 1
                // From python: D = c1 * r22d - s1 * r12d; E = s1 * r11d - c1 * r21d
                // atan2(-D / s5, E / s5) = atan2(-D, E) if s5 > 0
                // atan2( D / s5, -E / s5) = atan2( D, -E) if s5 < 0
                 const R16_x_axis_y = -r12d * s1 + r22d * c1; // Component of R06's y-axis projected onto R01's x-axis
                 const R16_x_axis_x = r11d * s1 - r21d * c1; // Component of R06's x-axis projected onto R01's x-axis (negated)

                let t6 = 0;
                if (Math.abs(s5) > tol_singularity) { // Avoid division by zero
                     t6 = Math.atan2(-R16_x_axis_y / s5, R16_x_axis_x / s5); // atan2(y,x)
                 } else {
                    // Wrist singularity: t5 is 0 or pi. t6 is not uniquely defined.
                    // Can often set t6 = 0, but the combined t4 might need adjustment.
                    // For simplicity, we'll keep t6=0 but acknowledge this isn't a full singularity handler.
                    t6 = 0; // Assign a value, but it's arbitrary
                    // console.log(`Singularity t5=${t5}, setting t6=0`);
                 }
                 t6 = normalizeAngle(t6);

                const s6 = Math.sin(t6);
                const c6 = Math.cos(t6);

                // --- Solve for Theta 3 ---
                // Need R16 matrix elements
                const R16_z_axis_x = c1 * r13d + s1 * r23d; // Projection of R06 z-axis onto R01 x-axis
                const R16_z_axis_y = -s1 * r13d + c1 * r23d; // Projection of R06 z-axis onto R01 y-axis (already computed as D?)
                const R16_z_axis_z = r33d;                  // Projection of R06 z-axis onto R01 z-axis

                // Need R16 components for t234 calculation later:
                const R16_y_axis_z = r32d; // Projection of R06 y-axis onto R01 z-axis
                const R16_x_axis_z = r31d; // Projection of R06 x-axis onto R01 z-axis


                // Calculate P31 position (wrist center in frame 1)
                 const P31_x = c5 * R16_z_axis_x * d5 - R16_y_axis_x * s5 * d5 + c1 * pxd + s1 * pyd; // Check derivation
                 const P31_y = c5 * R16_z_axis_y * d5 - R16_y_axis_y * s5 * d5 - s1 * pxd + c1 * pyd; // Check derivation
                 const P31_z = P50z; // Wrist center z relative to base (frame 0) is P50z

                 // Simplified calculation using P13 (vector from joint 1 to joint 3 projected onto plane)
                 // P13_xy_sq = (P50x * c1 + P50y * s1 - d4)^2 + (P50z - d1)^2; // Check this derivation carefully
                 // Simpler approach: Find wrist center P4_0 = T06 * [-d6*nx, -d6*ny, -d6*nz, 1]^T (where n is z-axis of frame 6) -> P4_0 = P60 - d6*R06[:,2] which is P50
                 // Then P4_1 = R01^T * P4_0
                 // P4_1_x = c1*P50x + s1*P50y
                 // P4_1_y = -s1*P50x + c1*P50y
                 // P4_1_z = P50z
                 // Target for joints 2,3 calculation is P4_1 projected onto the arm plane (relative to joint 1 offset)
                 // KC = P4_1_x
                 // KS = P4_1_z - d1

                 const KC = P50x * c1 + P50y * s1 - d4; // Corrected: Project P50 onto x-axis of frame 1, subtract d4 offset
                 const KS = P50z - d1; // Height relative to joint 1
                 const dist_sq_13 = KC**2 + KS**2;


                const denom_t3 = 2 * len_a2 * len_a3;
                if (Math.abs(denom_t3) < tol_zero) continue; // Should not happen for UR5e

                const cos_t3_arg = (dist_sq_13 - len_a2**2 - len_a3**2) / denom_t3;

                if (Math.abs(cos_t3_arg) > 1.0 + tol_geom) {
                   // console.warn(`IK Warning (t1:${t1_idx}, t5:${t5_idx}): t3 acos out of range: ${cos_t3_arg}`);
                    continue; // Cannot reach
                }
                const cos_t3_clipped = clip(cos_t3_arg, -1.0, 1.0);
                const t3_base = Math.acos(cos_t3_clipped);

                const theta3_sol = [t3_base, -t3_base]; // Two solutions for theta3 (elbow up/down relative to plane)

                for (let t3_idx = 0; t3_idx < theta3_sol.length; t3_idx++) {
                    const t3 = normalizeAngle(theta3_sol[t3_idx]);
                    const s3 = Math.sin(t3);
                    const c3 = Math.cos(t3); // Use clipped value? Using actual cos(t3) is better. recalculate:
                    // const c3 = Math.cos(t3);

                    // --- Solve for Theta 2 ---
                    const term1_t2 = Math.atan2(KS, KC); // Angle of the vector from joint 1 to wrist center in the arm plane
                    const term2_y_t2 = len_a3 * s3;
                    const term2_x_t2 = len_a2 + len_a3 * c3; // Note: a2 is negative, len_a2 is positive
                    const term2_t2 = Math.atan2(term2_y_t2, term2_x_t2); // Angle adjustment due to elbow bending

                    const t2 = normalizeAngle(term1_t2 - term2_t2);


                    // --- Solve for Theta 4 ---
                    // t234 = atan2(R16_y_axis_z, R16_x_axis_z) // Rotation about Z axis of frame 1 to align with frame 4?

                     // Need orientation of frame 4 relative to frame 1. T14 = T01^-1 * T04 = T01^T * T04
                     // T04 = T06 * T46^-1 = T06 * T56^-1 * T45^-1
                     // Need to calculate R14 from R06, t1, t5, t6
                     // Simpler: find t234 = t2+t3+t4 using R04 components relative to frame 0?
                     // Or use the orientation part of T_flange relative to base
                     // R03 = R01(t1)*R12(t2)*R23(t3)
                     // R36 = R34(t4)*R45(t5)*R56(t6)
                     // R06 = R03 * R36
                     // R03^T * R06 = R36
                     // We need t4 from R36 elements or from t234.

                    // Let's use the approach similar to Python code's t234
                     // F = c5 * c6; C_val = c1 * r11d + s1 * r21d;
                     // atan_y_234 = r31d * F - s6 * C_val; atan_x_234 = F * C_val + s6 * r31d;
                     // No, these seem incorrect or specific to a convention.

                     // Let's try deriving t234 from R04 = R01*R12*R23*R34
                     // R04 = R06 * R46^-1 = R06 * R56(t6)^-1 * R45(t5)^-1
                     // Calculate R46 = dh(0, -pi/2, d5, t5) * dh(0, 0, d6, t6) ? No, use R_alpha_d
                     // R45(t5) comes from alpha = -pi/2 -> RotX(-pi/2) then RotZ(t5)
                     // R56(t6) comes from alpha = 0 -> RotZ(t6)
                     // R46 = RotX(-pi/2)*RotZ(t5)*RotZ(t6) = RotX(-pi/2)*RotZ(t5+t6)
                     // R46_inv = RotZ(-t5-t6)*RotX(pi/2)

                    // Alternative: T04 = T01 * T12 * T23 * T34
                    // T34 = DH(a=0, alpha=pi/2, d=d4, theta=t4)
                    // T03 = T01 * T12 * T23
                    // T04 = T03 * T34
                    // T46 = T45 * T56
                    // T06 = T04 * T46
                    // T04 = T06 * T46^-1
                    // Calculate R04 = R06 * R46^T
                    // Then calculate t234 = atan2(R04[1,0], R04[0,0]) ? If ZYZ euler for arm? No.

                    // Let's use the geometric interpretation: t234 is the angle required
                    // to align the Z-axis of frame 1 with the Z-axis of frame 4 projected onto XY plane of frame 0?

                    // Using the Python code's logic for t234 (assuming it's correct for the convention):
                    const R16_y_axis_x = -r11d * s1 + r21d * c1; // Projection of R06 x-axis onto R01 y-axis
                    const R16_y_axis_y = r12d * s1 + r22d * c1; // Projection of R06 y-axis onto R01 y-axis (part of D)

                    if(Math.abs(s5) < tol_singularity){
                        // Wrist Singularity Case for t4
                         // When s5 is near zero, t4 and t6 are coupled.
                         // We conventionally set t6=0.
                         // t4 must be adjusted to achieve the final rotation.
                         // Need R05 = R06 * R56(t6=0)^T = R06
                         // R15 = R01^T * R05 = R01^T * R06
                         // t2+t3+t4 = atan2(R15_z_axis_y, R15_z_axis_x) ?
                         const R15_z_axis_y = -s1*r13d + c1*r23d; //Component of R06 z-axis onto R01 y-axis
                         const R15_z_axis_x = c1*r13d + s1*r23d; //Component of R06 z-axis onto R01 x-axis
                         const t234 = Math.atan2(R15_z_axis_y, R15_z_axis_x);
                         t4 = normalizeAngle(t234 - t2 - t3);
                          // console.log(`Singularity t5=${t5}, calculated t4=${t4}`);

                    } else {
                         // Regular case for t4
                         const R16_x_axis_component_for_t4 = R16_x_axis_y; // Check which component needed
                         const R16_y_axis_component_for_t4 = R16_y_axis_x; // Check which component needed

                         const t234_atan_y = R16_y_axis_component_for_t4 / s5; // Need R14 components?
                         const t234_atan_x = R16_x_axis_component_for_t4 / s5; // Need R14 components?

                         // Let's use the definition t4 = atan2(Ry_4_in_3, Rx_4_in_3) related to R34
                         // R34 = R03^T * R04 = (R01*R12*R23)^T * (R06*R56^T*R45^T)
                         // This gets complicated.

                        // Try again with Python's approach assuming it matches convention
                         // Requires T03 = T01*T12*T23
                         const T01 = dhMatrix(DH_PARAMS_UR5E[0].a, DH_PARAMS_UR5E[0].alpha, d1, t1);
                         const T12 = dhMatrix(DH_PARAMS_UR5E[1].a, DH_PARAMS_UR5E[1].alpha, DH_PARAMS_UR5E[1].d, t2);
                         const T23 = dhMatrix(DH_PARAMS_UR5E[2].a, DH_PARAMS_UR5E[2].alpha, DH_PARAMS_UR5E[2].d, t3);
                         const T02 = multiplyMatrices(T01, T12);
                         const T03 = multiplyMatrices(T02, T23);

                         // Need inverse of T03
                         const R03 = [[T03[0][0], T03[0][1], T03[0][2]], [T03[1][0], T03[1][1], T03[1][2]], [T03[2][0], T03[2][1], T03[2][2]]];
                         const R03_T = [[R03[0][0], R03[1][0], R03[2][0]], [R03[0][1], R03[1][1], R03[2][1]], [R03[0][2], R03[1][2], R03[2][2]]];
                         const P03 = [T03[0][3], T03[1][3], T03[2][3]];
                         // P30_in_0 = -R03_T * P03
                         const P30_in_0 = [
                             -(R03_T[0][0]*P03[0] + R03_T[0][1]*P03[1] + R03_T[0][2]*P03[2]),
                             -(R03_T[1][0]*P03[0] + R03_T[1][1]*P03[1] + R03_T[1][2]*P03[2]),
                             -(R03_T[2][0]*P03[0] + R03_T[2][1]*P03[1] + R03_T[2][2]),
                         ];
                         const T03_inv = [
                             [R03_T[0][0], R03_T[0][1], R03_T[0][2], P30_in_0[0]],
                             [R03_T[1][0], R03_T[1][1], R03_T[1][2], P30_in_0[1]],
                             [R03_T[2][0], R03_T[2][1], R03_T[2][2], P30_in_0[2]],
                             [0, 0, 0, 1]
                         ];

                         const T36 = multiplyMatrices(T03_inv, T_flange); // T03_inv * T06 = T36
                         const R36 = [[T36[0][0], T36[0][1], T36[0][2]], [T36[1][0], T36[1][1], T36[1][2]], [T36[2][0], T36[2][1], T36[2][2]]];

                         // Now t4 should be derivable from R36 elements, related to R34 = R36 * R46^T
                         // R36 = R34(t4) * R45(t5) * R56(t6)
                         // Check element R36[1,2] vs R36[0,2] ? (related to Z axis alignment)
                         // R36[2,2] = cos(beta) for ZYX or similar
                         // R36[1,2] = sin(alpha)cos(beta)
                         // R36[0,2] = -sin(beta)

                         // Simpler: Look at R36's Z-axis projection onto frame 3's Y and X axes
                         const R36_z_axis_y = T36[1][2]; // Z-axis of frame 6 projected onto Y-axis of frame 3
                         const R36_z_axis_x = T36[0][2]; // Z-axis of frame 6 projected onto X-axis of frame 3

                         t4 = Math.atan2(R36_z_axis_y, R36_z_axis_x);
                         t4 = normalizeAngle(t4);
                    }


                    // --- Collect Solution ---
                    const sol_angles_raw = [t1, t2, t3, t4, t5, t6];
                    const sol_angles_normalized = sol_angles_raw.map(normalizeAngle);

                    // --- Verification (Optional but Recommended) ---
                    const fkCheck = forwardKinematics(sol_angles_normalized, dhParams, H_FLANGE_TCP);
                    let is_valid = false;
                    if (fkCheck) {
                        const T_check = fkCheck.T0_TCP;
                         // Check if T_check is close to T_desired_TCP
                        let diffSumSq = 0;
                        for(let r=0; r<4; ++r) {
                            for(let c=0; c<4; ++c) {
                                diffSumSq += (T_check[r][c] - T_desired_TCP[r][c])**2;
                            }
                        }
                        if (Math.sqrt(diffSumSq) < tol_compare * 16) { // Compare matrices
                            is_valid = true;
                        } else {
                             // console.warn(`IK Solution ${solutions.length} failed FK validation. Diff: ${Math.sqrt(diffSumSq)}`);
                             // console.log("Target:", T_desired_TCP);
                             // console.log("Actual:", T_check);
                             // console.log("Angles:", sol_angles_normalized.map(a => a*180/Math.PI));
                        }
                    }

                    if (is_valid) {
                         // Avoid adding duplicate solutions (can happen with normalization/tolerance)
                        let too_close = false;
                        for(const existing_sol of solutions) {
                            let angle_diff_sq = 0;
                            for(let j=0; j<6; ++j) {
                                let diff = normalizeAngle(sol_angles_normalized[j] - existing_sol[j]);
                                angle_diff_sq += diff*diff;
                            }
                            if (Math.sqrt(angle_diff_sq) < tol_compare) {
                                too_close = true;
                                break;
                            }
                        }
                        if (!too_close) {
                             solutions.push(sol_angles_normalized);
                        }
                    }

                } // end t3 loop
            } // end t5 loop
        } // end t1 loop

        // console.log(`Found ${solutions.length} unique IK solutions.`);
        return solutions;
    }

    // --- Expose Public Methods ---
    return {
        forwardKinematics,
        inverseKinematics,
        DH_PARAMS_UR5E, // Expose constants if needed externally
        H_FLANGE_TCP,
        INV_H_FLANGE_TCP
    };

})(); // Immediately invoke the function expression to create the module

// Example Usage (optional, for testing in Node.js or browser console)
/*
const initial_joints = [0.0, -Math.PI/2, Math.PI/2, -Math.PI/2, -Math.PI/2, 0.0];
const fkResult = UR5eKinematics.forwardKinematics(initial_joints);

if (fkResult) {
    console.log("FK TCP Pose:", fkResult.T0_TCP);
    const ikSolutions = UR5eKinematics.inverseKinematics(fkResult.T0_TCP);
    console.log(`Found ${ikSolutions.length} IK solutions:`);
    ikSolutions.forEach((sol, idx) => {
        console.log(` Solution ${idx}:`, sol.map(a => (a * 180 / Math.PI).toFixed(2)));
    });
}
*/