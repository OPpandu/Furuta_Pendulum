%constants%
mp=1; % mass of pendulum
d=0.5; %distance of COM of pendulum from piviot
Ip=1; %MOI of pendulum about COM;
Bp=0; %damping coefficient of pendulum
Br=0; %damping coefficient of arm
l=1; %length of arm
Ir=1; %MOI of arm wrt piviot
g=9.81; %gravity


a=mp*l*l+Ir+mp*d*d;
b=mp*l*d;
C=Ip+mp*d*d;
D=mp*d*g;

A=[0 0 1 0;
   0 0 0 1;
   (a*D/(a*C-b*b)) (0) (-a*Bp/(a*C-b*b)) (-b*Br/(a*C-b*b));
   (b*D/(a*C-b*b)) (0) (-b*Bp/(a*C-b*b)) (-C*Br/(a*C-b*b))];

B=[0;0;b/(a*C-b*b);C/(a*C-b*b)];

desired_poles = [-2, -3, -0.4, -0.5]; 


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