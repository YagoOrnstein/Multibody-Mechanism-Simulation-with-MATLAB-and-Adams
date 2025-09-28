function a = vectangle360(v1, v2)
    % vectangle360 computes the signed angle (in radians) between two 2D vectors in the range [0, 2*pi].
    %
    % Inputs:
    %   v1 - First 2D vector.
    %   v2 - Second 2D vector.
    %
    % Outputs:
    %   a - The signed angle (in radians) between v1 and v2, measured counterclockwise.

    % Extend 2D vectors to 3D by adding a zero z-component
    v1 = [v1; 0];
    v2 = [v2; 0];

    % Define the normal vector for the plane (z-axis)
    n = [0; 0; 1];

    % Compute the cross product of v1 and v2
    x = cross(v1, v2);

    % Compute the magnitude of the cross product, adjusted by the sign of the z-component
    c = sign(dot(x, n)) * norm(x);

    % Compute the angle using atan2, incorporating the cross product magnitude and dot product
    a = atan2(c, dot(v1, v2));

    % Ensure the angle is in the range [0, 2*pi]
    if a < 0
        a = a + 2 * pi;
    end
end
