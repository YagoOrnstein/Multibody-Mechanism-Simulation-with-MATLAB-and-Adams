function [T, k] = mainCalc()
    % mainCalc performs the kinematic solution for position, velocity, and acceleration
    % of a multibody system over a specified time interval.
    %
    % Outputs:
    %   T - Time vector.
    %   k - Matrix containing position (Q), velocity (DQ), and acceleration (D2Q) solutions.
    %
    % Notes:
    %   This function solves kinematics iteratively using Newton-Raphson for position,
    %   and analytical methods for velocity and acceleration.

    % Load initial configuration and local vectors
    [pos_init, CofM] = Init_config();
    vect = Local_Vector();

    % FIND ALL ANGLES OF BODIES TO THEIR RESPECTIVE REFERENCE FRAMES

    fi_init = zeros(10, 1);
    x_unit = [1; 0];
    fi_init(10) = vectangle360(x_unit, vect.D109); % Angle for Body 10
    fi_init(9) = vectangle360(x_unit, vect.D109);  % Angle for Body 9
    fi_init(8) = vectangle360(x_unit, vect.D87);   % Angle for Body 8
    fi_init(7) = vectangle360(x_unit, vect.D87);   % Angle for Body 7

    % CREATE INITIAL q VECTOR (POSITIONS AND ORIENTATIONS)

    q_init = zeros(30, 1);

    % Assign initial positions and orientations to q_init
    q_init(1:2) = pos_init.A;         % C1 - Position in global frame (t=0)
    q_init(3) = fi_init(1, 1);        % Orientation of Body 1
    q_init(4:5) = pos_init.J;         % C2
    q_init(6) = fi_init(2, 1);        % Orientation of Body 2
    q_init(7:8) = pos_init.L;         % C3
    q_init(9) = fi_init(3);           % Orientation of Body 3
    q_init(10:11) = pos_init.I;       % C4
    q_init(12) = fi_init(4);          % Orientation of Body 4
    q_init(13:14) = pos_init.B;       % C5
    q_init(15) = fi_init(5);          % Orientation of Body 5
    q_init(16:17) = pos_init.D;       % C6
    q_init(18) = fi_init(6);          % Orientation of Body 6
    q_init(19:20) = pos_init.M;       % C7
    q_init(21) = fi_init(7);          % Orientation of Body 7
    q_init(22:23) = pos_init.N;       % C8
    q_init(24) = fi_init(8);          % Orientation of Body 8
    q_init(25:26) = pos_init.G;       % C9
    q_init(27) = fi_init(9);          % Orientation of Body 9
    q_init(28:29) = pos_init.H;       % C10
    q_init(30) = fi_init(10);         % Orientation of Body 10

    % INITIALIZATION

    q = q_init;
    dq = zeros(30, 1);     % Initial velocities
    d2q = zeros(30, 1);    % Initial accelerations
    counter = 0;           % Counter for iterations
    dt = 0.1;              % Time step

    % Preallocate result matrices
    T = [];
    Q = [];
    DQ = [];
    D2Q = [];

    % SOLVE KINEMATICS OVER TIME

    for t = 0:dt:5
        % Initial approximation for Newton-Raphson method
        q0 = q + dq * dt + 0.5 * d2q * dt^2;

        % Solve position problem using Newton-Raphson
        q = NewtonRaphsonCalc(q0, t);

        % Solve velocity problem
        dq = velocityCalc(q, t);

        % Solve acceleration problem
        d2q = accelerationCalc(dq, q, t);

        % Store results
        counter = counter + 1;
        T(1, counter) = t;
        Q(:, counter) = q;
        DQ(:, counter) = dq;
        D2Q(:, counter) = d2q;
    end

    % Combine results into a single matrix
    k = [Q DQ D2Q];

    % Output results to console (for debugging or inspection)
    fprintf('Final position vector Q:\n');
    disp(Q);
end
