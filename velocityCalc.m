function [dq, Fq] = velocityCalc(q, t)
    % velocityCalc computes the velocities of a multibody system at a given time.
    % This function solves the velocity problem, which must follow the solution of the position problem.
    %
    % Inputs:
    %   q - Absolute coordinates of the multibody system.
    %   t - Time at which the solution is sought.
    %
    % Outputs:
    %   dq - Time derivatives of the absolute coordinates (velocities).
    %   Fq - Jacobian matrix of the constraint equations.

    % Compute time derivatives for driving constraints
    dMotion_87 = 1.5 * 0.05 * cos(1.5 * t);  % Time derivative of motion for joint 8-7
    dMotion_109 = 1.5 * 0.05 * cos(1.5 * t); % Time derivative of motion for joint 10-9

    % Right-hand side of the velocity equations
    Ft = [zeros(28, 1); -dMotion_87; -dMotion_109];

    % Calculate the Jacobian matrix of the constraints
    Fq = JacobianCalc(q);

    % Solve for velocities using the linear system Fq * dq = Ft
    dq = -Fq \ Ft;
end
