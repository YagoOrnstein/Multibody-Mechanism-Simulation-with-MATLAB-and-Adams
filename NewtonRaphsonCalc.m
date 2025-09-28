function q = NewtonRaphsonCalc(q0, t)
    % NewtonRaphsonCalc solves a set of nonlinear equations using the Newton-Raphson method.
    %
    % Inputs:
    %   q0 - Initial approximation of the solution.
    %   t  - Current time instant.
    %
    % Outputs:
    %   q - Solution vector satisfying the constraints.
    %
    % Notes:
    %   - The set of nonlinear equations must be defined in constraintCalc.
    %   - The constraint Jacobian matrix must be defined in JacobianCalc.

    % Initialize the solution vector with the initial approximation
    q = q0;

    % Calculate initial constraint function values
    F = constraintCalc(q, t);

    % Set iteration parameters
    iter = 1;          % Iteration counter
    max_iter = 25;     % Maximum number of iterations
    tol = 1e-10;       % Convergence tolerance

    % Newton-Raphson iteration loop
    while (norm(F) > tol) && (iter <= max_iter)
        % Recalculate constraints and Jacobian matrix
        F = constraintCalc(q, t);
        Fq = JacobianCalc(q);

        % Update solution using the Newton-Raphson formula
        q = q - Fq \ F; % Solve the linear system Fq * delta_q = F

        % Increment iteration counter
        iter = iter + 1;
    end

    % Check for convergence
    if iter > max_iter
        disp('ERROR: No convergence after 25 iterations');
        q = q0; % Revert to initial approximation
    end
end
