function Jacobian = JacobianCalc(q)
    % JacobianCalc computes the Jacobian matrix for the multibody system.
    % The Jacobian matrix is used to solve the system of equations for constraints.
    %
    % Inputs:
    %   q - Absolute coordinates of the multibody system.
    %
    % Output:
    %   Jacobian - The calculated Jacobian matrix.

    % Initialize local vectors
    vect = Local_Vector();

    % Assign positions and orientations from the state vector q
    r0 = [0; 0];       % Fixed reference origin
    r1 = q(1:2);       fi_1 = q(3); 
    r2 = q(4:5);       fi_2 = q(6);
    r3 = q(7:8);       fi_3 = q(9);
    r4 = q(10:11);     fi_4 = q(12);
    r5 = q(13:14);     fi_5 = q(15);
    r6 = q(16:17);     fi_6 = q(18);
    r7 = q(19:20);     fi_7 = q(21);
    r8 = q(22:23);     fi_8 = q(24);
    r9 = q(25:26);     fi_9 = q(27);
    r10 = q(28:29);    fi_10 = q(30);

    % Compute rotation matrices for all bodies
    R_0 = eye(2);  % Fixed reference frame
    R_01 = Rot(fi_1); R_02 = Rot(fi_2); R_03 = Rot(fi_3);
    R_04 = Rot(fi_4); R_05 = Rot(fi_5); R_06 = Rot(fi_6);
    R_07 = Rot(fi_7); R_08 = Rot(fi_8); R_09 = Rot(fi_9);
    R_010 = Rot(fi_10);

    % Initialize the Jacobian matrix
    Jacobian = zeros(30, 30);

    % Skew-symmetric matrix for cross products
    Omega = [0, -1; 1, 0];

    % Revolute joint constraints (Eq. 5.30-33)
    Jacobian(1:2, 7:8) = eye(2);                % 0-3 (q7)
    Jacobian(1:2, 9) = Omega * R_03 * vect.S30; % q9

    Jacobian(3:4, 7:8) = eye(2);                % 3-4 (q7)
    Jacobian(3:4, 9) = Omega * R_03 * vect.S34; % q9
    Jacobian(3:4, 10:11) = -eye(2);             % q10
    Jacobian(3:4, 12) = -Omega * R_04 * vect.S43; % q12

    Jacobian(5:6, 1:2) = -eye(2);               % 4-1 (q1)
    Jacobian(5:6, 3) = -Omega * R_01 * vect.S14; % q3
    Jacobian(5:6, 10:11) = eye(2);              % q10
    Jacobian(5:6, 12) = Omega * R_04 * vect.S41; % q12

    Jacobian(7:8, 1:2) = eye(2);                % 1-5 (q1)
    Jacobian(7:8, 3) = Omega * R_01 * vect.S15; % q3
    Jacobian(7:8, 13:14) = -eye(2);             % q13
    Jacobian(7:8, 15) = -Omega * R_05 * vect.S51; % q15

    Jacobian(9:10, 4:5) = -eye(2);              % 5-2 (q3)
    Jacobian(9:10, 6) = -Omega * R_02 * vect.S25; % q6
    Jacobian(9:10, 13:14) = eye(2);             % q13
    Jacobian(9:10, 15) = Omega * R_05 * vect.S52; % q15

    Jacobian(11:12, 4:5) = eye(2);              % 2-7 (q3)
    Jacobian(11:12, 6) = Omega * R_02 * vect.S27; % q6
    Jacobian(11:12, 19:20) = -eye(2);           % q19
    Jacobian(11:12, 21) = -Omega * R_07 * vect.S72; % q21

    Jacobian(13:14, 4:5) = eye(2);              % 2-3 (q3)
    Jacobian(13:14, 6) = Omega * R_02 * vect.S23; % q6
    Jacobian(13:14, 7:8) = -eye(2);             % q7
    Jacobian(13:14, 9) = -Omega * R_03 * vect.S32; % q9

    Jacobian(15:16, 7:8) = eye(2);              % 3-9 (q7)
    Jacobian(15:16, 9) = Omega * R_03 * vect.S39; % q9
    Jacobian(15:16, 25:26) = -eye(2);           % q25
    Jacobian(15:16, 27) = -Omega * R_09 * vect.S93; % q27

    Jacobian(17:18, 10:11) = eye(2);            % 4-6 (q10)
    Jacobian(17:18, 12) = Omega * R_04 * vect.S46; % q12
    Jacobian(17:18, 16:17) = -eye(2);           % q16
    Jacobian(17:18, 18) = -Omega * R_06 * vect.S64; % q18

    Jacobian(19:20, 4:5) = -eye(2);             % 6-2 (q3)
    Jacobian(19:20, 6) = -Omega * R_02 * vect.S26; % q6
    Jacobian(19:20, 16:17) = eye(2);            % q16
    Jacobian(19:20, 18) = Omega * R_06 * vect.S62; % q18

    Jacobian(21:22, 22:23) = eye(2);            % 0-8 (q22)
    Jacobian(21:22, 24) = Omega * R_08 * vect.S80; % q24

    Jacobian(23:24, 28:29) = eye(2);            % 0-10 (q28)
    Jacobian(23:24, 30) = Omega * R_010 * vect.S100; % q30

    % Angular constraint (8-7)
    Jacobian(25, 21) = -1;                       % q21
    Jacobian(25, 24) = 1;                        % q24

    % Translational constraint (8-7)
    [dF_dr_i, dF_dphi_i, dF_dr_j, dF_dphi_j] = Jacobian_TransJ(r8, R_08, r7, R_07, vect.S87, vect.W87);
    Jacobian(26, 19:20) = dF_dr_j;              % q19
    Jacobian(26, 21) = dF_dphi_j;              % q21
    Jacobian(26, 22:23) = dF_dr_i;             % q22
    Jacobian(26, 24) = dF_dphi_i;              % q24

    % Angular constraint (10-9)
    Jacobian(27, 27) = -1;                       % q27
    Jacobian(27, 30) = 1;                        % q30

    % Translational constraint (10-9)
    [dF_dr_i, dF_dphi_i, dF_dr_j, dF_dphi_j] = Jacobian_TransJ(r10, R_010, r9, R_09, vect.S109, vect.W109);
    Jacobian(28, 25:26) = dF_dr_j;             % q25
    Jacobian(28, 27) = dF_dphi_j;              % q27
    Jacobian(28, 28:29) = dF_dr_i;             % q28
    Jacobian(28, 30) = dF_dphi_i;              % q30

    % Driving constraint (8-7)
    [dFt_dr_i, dFt_dphi_i, dFt_dr_j, dFt_dphi_j] = Jacobian_TransJ_Drive(r8, R_08, r7, R_07, vect.S87, vect.U87);
    Jacobian(29, 19:20) = dFt_dr_j;            % q19
    Jacobian(29, 21) = dFt_dphi_j;            % q21
    Jacobian(29, 22:23) = dFt_dr_i;           % q22
    Jacobian(29, 24) = dFt_dphi_i;            % q24

    % Driving constraint (10-9)
    [dFt_dr_i, dFt_dphi_i, dFt_dr_j, dFt_dphi_j] = Jacobian_TransJ_Drive(r10, R_010, r9, R_09, vect.S109, vect.U109);
    Jacobian(30, 25:26) = dFt_dr_j;           % q25
    Jacobian(30, 27) = dFt_dphi_j;            % q27
    Jacobian(30, 28:29) = dFt_dr_i;           % q28
    Jacobian(30, 30) = dFt_dphi_i;            % q30
end

%% Supporting Jacobian Functions

function [dF_dr_i, dF_dfi_i, dF_dr_j, dF_dfi_j] = Jacobian_RevJ(Ri, SA, Rj, SB)
    % Jacobian_RevJ calculates Jacobian entries for a revolute joint constraint.
    Omega = [0 -1; 1 0];
    dF_dr_i = eye(2);
    dF_dfi_i = Omega * Ri * SA;
    dF_dr_j = -eye(2);
    dF_dfi_j = -Omega * Rj * SB;
end

function [dF_dr_i, dF_dphi_i, dF_dr_j, dF_dphi_j] = Jacobian_TransJ(r_i, Ri, r_j, Rj, SA, v)
    % Jacobian_TransJ calculates Jacobian entries for a translational joint constraint.
    Omega = [0 -1; 1 0];
    dF_dr_i = -(Rj * v)';
    dF_dphi_i = -(Rj*v)' * Omega * Ri * SA;
    dF_dr_j = (Rj*v)';
    dF_dphi_j = -(Rj*v)' * Omega * (r_j - r_i - Ri * SA);
end

function [dFt_dr_i, dFt_dphi_i, dFt_dr_j, dFt_dphi_j] = Jacobian_TransJ_Drive(r_i, Ri, r_j, Rj, SA, u)
    % Jacobian_TransJ_Drive calculates Jacobian entries for a driving translational constraint.
    Omega = [0 -1; 1 0];
    dFt_dr_i = -(Rj * u)';
    dFt_dphi_i = -(Rj*u)' * Omega * Ri * SA;
    dFt_dr_j = (Rj*u)';
    dFt_dphi_j = -(Rj*u)' * Omega * (r_j - r_i - Ri * SA);
end
