function R = Rot(fi)
    % Rot generates a 2x2 rotation matrix for a given angle in radians.
    %
    % Inputs:
    %   fi - The rotation angle in radians.
    %
    % Outputs:
    %   R - The 2x2 rotation matrix.
    %
    % The rotation matrix is defined as:
    %   R = [cos(fi) -sin(fi);
    %        sin(fi)  cos(fi)]

    R = [cos(fi) -sin(fi); sin(fi) cos(fi)];
end
