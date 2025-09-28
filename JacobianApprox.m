clear all

    %Input data
    [pos_init, CofM] = Init_config();
    vect = Local_Vector();
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %FIND ALL ANGLES OF BODIES TO THEIR RESPECTIVE REFERENCE FRAMES

    fi_init = zeros(10,1);
    x_unit = [1;0];
    fi_init(10) = vectangle360(x_unit,vect.D109);
    fi_init(9) = vectangle360(x_unit,vect.D109);
    
    fi_init(8) = vectangle360(x_unit,vect.D87);
    fi_init(7) = vectangle360(x_unit,vect.D87);
    
   
    %////////////////////////////////////////////
    %Create initial q vector
    
    q_init = zeros(30,1);
    
    %q1
    q_init(1:2) = pos_init.A; %C1 / Position in global frame at start (t=0)
    %q2
    q_init(3) = fi_init(1,1);
    %q3
    q_init(4:5) = pos_init.J;%C2
    %q4
    q_init(6) = fi_init(2,1);
    %q5
    q_init(7:8) = pos_init.L;%C3
    %q6
    q_init(9) = fi_init(3);
    %q7
    q_init(10:11) = pos_init.I;%C4
    %q8
    q_init(12) = fi_init(4);
    %q9
    q_init(13:14) = pos_init.B;%C5
    %q10
    q_init(15) = fi_init(5);
    %q11
    q_init(16:17) = pos_init.D;%C6
    %q12
    q_init(18) = fi_init(6);
    %q13
    q_init(19:20) = pos_init.M;%C7
    %q14
    q_init(21) = fi_init(7);
    %q15
    q_init(22:23) = pos_init.N;%C8
    %q16
    q_init(24) = fi_init(8);
    %q17
    q_init(25:26) = pos_init.G;%C9
    %q18
    q_init(27) = fi_init(9);
    %q19
    q_init(28:29) = pos_init.H;%C10
    %q20
    q_init(30) = fi_init(10);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q = q_init;
dq=zeros(size(q)); d2q=zeros(size(q));
J = zeros(30, 30);
dt=0.1; % Time step

q0=q+dq*dt+0.5*d2q*dt^2;

delta = 0.01;
delta_vec = zeros(30, 1);
delta_vec(1) = delta;
for i = 1:30
    q_i = q0 + delta_vec;
    c_i = ( constraintCalc(q_i, 0) - constraintCalc(q0, 0) ) / delta;
    approx_F(:, i) = round(c_i, 4);
    delta_vec = circshift(delta_vec, 1);
end

calcul_F=JacobianCalc(constraintCalc(q0,0.1));
fprintf("approx_det: %d while calcul_det: %d\n", det(approx_F), det(calcul_F))