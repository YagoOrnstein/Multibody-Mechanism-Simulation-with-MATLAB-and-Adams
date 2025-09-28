function [vect] = Local_Vector()
    % Local_Vector defines the local frame vectors for joints on the bodies in the multibody system.
    % These vectors are used to calculate constraint forces and motions.
    %
    % Output:
    %   vect - A structure containing local frame vectors.

    % Load initial configuration and center of mass data
    [pos_init] = Init_config();

    % Nomenclature: 
    %   SXY: Vector from joint X to joint Y within the body
    %   WXY: Perpendicular vector for translational joints
    %   UXY: Parallel vector for translational joints

    % Global to H (SA)
    vect.S010 = pos_init.H;
    % Local frame C10 to H (SB)
    vect.S100 = zeros(2, 1);

    % Global to N
    vect.S08 = pos_init.N;
    % Local frame C8 to N
    vect.S80 = zeros(2, 1);

    % Global to L
    vect.S03 = pos_init.L;
    % Local frame C8 to L
    vect.S30 = zeros(2, 1);

    % Ref_C3 to G
    vect.S39 = pos_init.G - pos_init.L;
    % Ref_C9 to G
    vect.S93 = zeros(2, 1);

    % Ref_C3 to F
    vect.S32 = pos_init.F - pos_init.L;
    % Ref_C2 to F
    vect.S23 = pos_init.F - pos_init.J;

    % Ref_C3 to I
    vect.S34 = pos_init.I - pos_init.L;
    % Ref_C4 to I
    vect.S43 = zeros(2, 1);

    % Ref_C4 to D
    vect.S46 = pos_init.D - pos_init.I;
    % Ref_C6 to D
    vect.S64 = zeros(2, 1);

    % Ref_C6 to E
    vect.S62 = pos_init.E - pos_init.D;
    % Ref_C2 to E
    vect.S26 = pos_init.E - pos_init.J;

    % Ref_C4 to A
    vect.S41 = pos_init.A - pos_init.I;
    % Ref_C1 to A
    vect.S14 = zeros(2, 1);

    % Ref_C1 to B
    vect.S15 = pos_init.B - pos_init.A;
    % Ref_C5 to B
    vect.S51 = zeros(2, 1);

    % Ref_C5 to J
    vect.S52 = pos_init.J - pos_init.B;
    % Ref_C2 to J
    vect.S25 = zeros(2, 1);

    % Ref_C2 to M
    vect.S27 = pos_init.M - pos_init.J;
    % Ref_C7 to M
    vect.S72 = zeros(2, 1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Translational Joint Vectors

    % Translational joint vectors (aligned with line of translation, thus zero)
    vect.S87 = zeros(2, 1);
    vect.S78 = zeros(2, 1);
    vect.S109 = zeros(2, 1);
    vect.S910 = zeros(2, 1);

    % Perpendicular vectors for translational joints (local frame orientation)
    vect.W87 = [0; 1];
    vect.W109 = [0; 1];

    % Parallel vectors for translational joints (local frame orientation)
    vect.U87 = [1; 0];
    vect.U109 = [1; 0];

    % Vectors along the line of translation (global frame orientation)
    vect.D87 = pos_init.M - pos_init.N;
    vect.D109 = pos_init.G - pos_init.H;
end
