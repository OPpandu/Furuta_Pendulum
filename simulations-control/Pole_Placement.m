%constants%
mp=1; % mass of pendulum
d=0.5; %distance of COM of pendulum from piviot
Ip=1; %MOI of pendulum about COM;
Bp=0; %damping coefficient of pendulum
Br=0; %damping coefficient of arm
l=1; %length of arm
Ir=10; %MOI of arm wrt piviot
g=9.81; %gravity
R = 2.0;    % Resistance
L = 0.01;   % Inductance
Kt = 0.1;   % Torque constant
Ke = 0.1;   % Back-EMF constant
Bm = 0.01;  % Motor damping 

a=mp*l*l+Ir+mp*d*d;
b=mp*l*d;
C=Ip+mp*d*d;
D=mp*d*g;
    torque= Kt*y(5) - Bm*y(3);

A=[0 0 1 0 0;
   0 0 0 1 0;
   (a*D/(a*C-b*b)) (0) (-a*Bp/(a*C-b*b)) ((-b*Br/(a*C-b*b))-Bm*b/(a*C-b*b)) Kt*b/(a*C-b*b);
   (b*D/(a*C-b*b)) (0) (-b*Bp/(a*C-b*b)) ((-C*Br/(a*C-b*b))-(Bm*C/(a*C-b*b))) C*Kt/(a*C-b*b);
   0 0 -Ke/L  0 -R/L];
   
B=[0;0;0;0;1/L];

desired_poles = [-2, -3, -0.4, -0.5, -0.9]; 


if rank(ctrb(A, B)) == size(A, 1)
    disp('System is controllable!');
    
    K = place(A, B, desired_poles);
    
    disp('State-feedback gain matrix K:');
    disp(K);
    
    A_cl = A - B*K;
    disp('Actual closed-loop poles:');
    disp(eig(A_cl)');
else
    error('System is uncontrollable! Re-check A/B matrices.');
end