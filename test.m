% Debug script for constraintCalc.m

% Clear workspace and initialize
clear;
clc;

% Load initial configuration
[pos_init, CofM] = Init_config();
vect = Local_Vector();

% Initialize q vector (position vector)
q = zeros(30, 1);

% Assign initial positions and orientations to q
q(1:2) = pos_init.A;       % Point A
q(3) = 0;                  % Angle 1
q(4:5) = pos_init.J;       % Point J
q(6) = 0;                  % Angle 2
q(7:8) = pos_init.L;       % Point L
q(9) = 0;                  % Angle 3
q(10:11) = pos_init.I;     % Point I
q(12) = 0;                 % Angle 4
q(13:14) = pos_init.B;     % Point B
q(15) = 0;                 % Angle 5
q(16:17) = pos_init.D;     % Point D
q(18) = 0;                 % Angle 6
q(19:20) = pos_init.M;     % Point M
q(21) = 0;                 % Angle 7
q(22:23) = pos_init.N;     % Point N
q(24) = 0;                 % Angle 8
q(25:26) = pos_init.G;     % Point G
q(27) = 0;                 % Angle 9
q(28:29) = pos_init.H;     % Point H
q(30) = 0;                 % Angle 10

% Set the time for testing
t = 0;  % Initial time

% Call the constraintCalc function
F_Constraint = constraintCalc(q, t);

% Display and debug individual constraints
disp('Debugging Constraints:');

% Loop through constraints
for i = 1:length(F_Constraint)
    fprintf('Constraint %d: %.6f\n', i, F_Constraint(i));
end

% Check which constraints are violated
violated_constraints = find(abs(F_Constraint) > 1e-6);
if isempty(violated_constraints)
    disp('All constraints are satisfied.');
else
    fprintf('Violated constraints at t = %f:\n', t);
    disp(violated_constraints);
    disp('Values of violated constraints:');
    disp(F_Constraint(violated_constraints));
end

% Debugging constraints 26 and 28
disp('Inputs for Constraint 26 (TransKin):');
disp('r8:'), disp(q(22:23));
disp('r7:'), disp(q(19:20));
disp('S87:'), disp(vect.S87);
disp('W87:'), disp(vect.W87);

disp('Inputs for Constraint 28 (TransKin):');
disp('r10:'), disp(q(28:29));
disp('r9:'), disp(q(25:26));
disp('S109:'), disp(vect.S109);
disp('W109:'), disp(vect.W109);

