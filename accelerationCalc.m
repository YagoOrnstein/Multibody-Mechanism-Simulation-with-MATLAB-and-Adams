function d2q = accelerationCalc(dq, q, t)
    % accelerationCalc calculates the accelerations of a multibody system.
    % This function solves the acceleration problem in a multibody system.
    % Note: Position and velocity solutions must be computed before calling this function.
    %
    % Inputs:
    %   dq - Derivatives of absolute coordinates with respect to time (velocities).
    %   q  - Absolute coordinates of the multibody system.
    %   t  - Time for which the acceleration solution is sought.
    %
    % Output:
    %   d2q - Second derivatives of absolute coordinates (accelerations).

    % Load local vectors for the system
    vect = Local_Vector();

    % Initialize position vectors and orientations for each body
    % These represent the positions and orientations of bodies in the system
    r0 = [0; 0];         fi_0 = 0;
    r1 = q(1:2);         fi_1 = q(3); 
    r2 = q(4:5);         fi_2 = q(6);
    r3 = q(7:8);         fi_3 = q(9);
    r4 = q(10:11);       fi_4 = q(12);
    r5 = q(13:14);       fi_5 = q(15);
    r6 = q(16:17);       fi_6 = q(18);
    r7 = q(19:20);       fi_7 = q(21);
    r8 = q(22:23);       fi_8 = q(24);
    r9 = q(25:26);       fi_9 = q(27);
    r10 = q(28:29);      fi_10 = q(30);

    % Compute rotation matrices for each body
    R_0 = eye(2);
    R_01 = Rot(fi_1); R_02 = Rot(fi_2); R_03 = Rot(fi_3); 
    R_04 = Rot(fi_4); R_05 = Rot(fi_5); R_06 = Rot(fi_6);
    R_07 = Rot(fi_7); R_08 = Rot(fi_8); R_09 = Rot(fi_9); 
    R_010 = Rot(fi_10);

    % Initialize velocity vectors and angular velocities for each body
    dr0 = [0; 0];        dfi_0 = 0;
    dr1 = dq(1:2);       dfi_1 = dq(3); 
    dr2 = dq(4:5);       dfi_2 = dq(6);
    dr3 = dq(7:8);       dfi_3 = dq(9);
    dr4 = dq(10:11);     dfi_4 = dq(12);
    dr5 = dq(13:14);     dfi_5 = dq(15);
    dr6 = dq(16:17);     dfi_6 = dq(18);
    dr7 = dq(19:20);     dfi_7 = dq(21);
    dr8 = dq(22:23);     dfi_8 = dq(24);
    dr9 = dq(25:26);     dfi_9 = dq(27);
    dr10 = dq(28:29);    dfi_10 = dq(30);

    % Define the skew-symmetric Omega matrix for cross products
    Omega = [0 -1; 1 0];

    % Calculate the right-hand side acceleration terms for constraints
    Gamma = zeros(30, 1);
    Gamma(1:2, 1)  = Accel_RevKin(R_03, dfi_3, vect.S30, R_0, dfi_0, vect.S03);
    Gamma(3:4, 1)  = Accel_RevKin(R_03, dfi_3, vect.S34, R_04, dfi_4, vect.S43);
    Gamma(5:6, 1)  = Accel_RevKin(R_04, dfi_4, vect.S41, R_01, dfi_1, vect.S14);
    Gamma(7:8, 1)  = Accel_RevKin(R_01, dfi_1, vect.S15, R_05, dfi_5, vect.S51);
    Gamma(9:10, 1) = Accel_RevKin(R_05, dfi_5, vect.S52, R_02, dfi_2, vect.S25);
    Gamma(11:12, 1) = Accel_RevKin(R_02, dfi_2, vect.S27, R_07, dfi_7, vect.S72);
    Gamma(13:14, 1) = Accel_RevKin(R_02, dfi_2, vect.S23, R_03, dfi_3, vect.S32);
    Gamma(15:16, 1) = Accel_RevKin(R_03, dfi_3, vect.S39, R_09, dfi_9, vect.S93);
    Gamma(17:18, 1) = Accel_RevKin(R_04, dfi_4, vect.S46, R_06, dfi_6, vect.S64);
    Gamma(19:20, 1) = Accel_RevKin(R_06, dfi_6, vect.S62, R_02, dfi_2, vect.S26);
    Gamma(21:22, 1) = Accel_RevKin(R_08, dfi_8, vect.S80, R_0, dfi_0, vect.S08);
    Gamma(23:24, 1) = Accel_RevKin(R_010, dfi_10, vect.S100, R_0, dfi_0, vect.S010);

    % Angular constraints (fixed to zero)
    Gamma(25, 1) = 0; % Angular constraint for 8-7
    Gamma(27, 1) = 0; % Angular constraint for 10-9

    % Translational constraints with driven motions
    d2Motion_87 = (1.5^2) * 0.05 * sin(1.5 * t);
    d2Motion_109 = (1.5^2) * 0.05 * sin(1.5 * t);
    Gamma(26, 1) = Accel_TransKin(r8, R_08, dr8, dfi_8, r7, R_07, dr7, dfi_7, vect.S87, vect.W87);
    Gamma(28, 1) = Accel_TransKin(r10, R_010, dr10, dfi_10, r9, R_09, dr9, dfi_9, vect.S109, vect.W109);
    Gamma(29, 1) = Accel_TransDrive(r8, R_08, dr8, dfi_8, r7, R_07, dr7, dfi_7, vect.S87, vect.U87, d2Motion_87);
    Gamma(30, 1) = Accel_TransDrive(r10, R_010, dr10, dfi_10, r9, R_09, dr9, dfi_9, vect.S109, vect.U109, d2Motion_109);

    % Calculate the Jacobian matrix of the constraints
    Fq = JacobianCalc(q);

    % Solve the system of equations to find accelerations
    d2q = Fq \ Gamma;
end

%% Function Definitions for Constraints

function [F_accel] = Accel_RevKin(Ri, dfi_i, SA, Rj, dfi_j, SB)
    % Accel_RevKin calculates acceleration for a revolute joint constraint.
    F_accel = Ri * SA * dfi_i^2 - Rj * SB * dfi_j^2;
end

function [F_accel] = Accel_TransKin(r_i, Ri, dr_i, dfi_i, r_j, Rj, dr_j, dfi_j, SA, v)
    % Accel_TransKin calculates acceleration for a translational joint constraint.
    % Eq. 5.58 (from Lecture 5)
    Omega = [0 -1; 1 0];
    F_accel = (Rj * v)' * (2 * Omega * (dr_j - dr_i) * dfi_j + (r_j - r_i) * dfi_j^2 - Ri * SA * (dfi_j - dfi_i)^2);
end

function [F_accel] = Accel_TransDrive(r_i, Ri, dr_i, dfi_i, r_j, Rj, dr_j, dfi_j, SA, u, F_motion)
    % Accel_TransDrive calculates driven translational constraints.
    % Modified Eq. 5.58 (from Lecture 5)
    Omega = [0 -1; 1 0];
    F_accel = (Rj * u)' * (2 * Omega * (dr_j - dr_i) * dfi_j + (r_j - r_i) * dfi_j^2 - Ri * SA * (dfi_j - dfi_i)^2) - F_motion;
end
