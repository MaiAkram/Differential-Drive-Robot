% MOBILE ROBOT MODEL PARAMETERS
r = 0.1;       % WHEEL RADIUS
L = 0.41;      % TRACK LENGTH
F_max = 1.0;   % MAXIMUNM ALLOWED WHEEL FORCE
Jw = 0.004;    % WHEEL MOMENT OF INERTIA
J = 0.0178;    % MOMENT OF INERTIA
mb = 1.25;     % BODY MASS
mw = 0.4;      % WHEEL MASS
m = mb + 2*mw; % TOTAL MASS

% GAINS
K1 = 5;
K2 = 10;
K3 = 10;

% PID
%Kp = 0.00032812; %8;
%Kd = 0.0030134;  %8;
%KI = 0.00060267; %2;
Kp = 8;
Kd = 8;
KI = 2;

Kpx = 0.37363;
Kdx = 1.686;
KIx = 0.093423;

% STATE SPACE
% x' = Ax + Bu
% y = Cx + Du
% x = [w_r w_l]
% y = [x y theta]
% u = [t1 t2]
M = [Jw+((r^2)/(L^2))*(m*(L^2)/4 + J) ((r^2)/(L^2))*(m*(L^2)/4 - J);
     ((r^2)/(L^2))*(m*(L^2)/4 - J)    Jw+((r^2)/(L^2))*(m*(L^2)/4 + J)]; % INERTIAL MATRIX
%V = [0 ((r^2)/(L^2))]; % CORIOLIS MATRIX
A = [0 0;
     0 0];
B_ = [1 0;
      0 1];
B = M\B_;
C = [r/2 r/2;
     0   0
     r/L -r/L];
D = [0 0;
     0 0;
     0 0];

syms theta s

mat = [cos(theta) 0;
       sin(theta) 0;
       0          1];
mat2 = [r/2 r/2;
        r/L -r/L];
mat3 = mat*mat2;
mat_inv = (transpose(mat)*mat)\transpose(mat);
mat3_inv = (transpose(mat3)*mat3)\transpose(mat3);

I = [1 0;
     0 1];
F = C*inv((s*I-A))*B+D;

