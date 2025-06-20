function V = Voltage(t, y)
    k = 1e+03 * [4.4803, -0.0000, 2.2416, -0.0023, 0.0000];
    V = dot(k, y);  % or: V = k * y'; if y is a column vector
end


%%differential equations
function dydt=odefun(t,y,mp,d,Ip,Bp,Br,l,Ir,g,R,L,Kt,Ke,Bm)
    
    A=(mp*l*l + mp*d*d*cos(y(1)) + Ir);
    B=mp*l*d*cos(y(1));
    C=(mp*y(3)*y(4)*d*d*sin(2*y(1))) + mp*d*l*sin(y(1))*y(3)*y(3);
    D=(mp*d*(d/2)*sin(2*(y(1)))) + mp*g*d*sin(y(1));
    E=mp*d*d+Ip;
    torque= Kt*y(5) - Bm*y(3);
    

    dydt=zeros(5,1);
    dydt(1)= y(3); %theta
    dydt(2)= y(4); %phi
    dydt(3)= ((torque - Br*y(4))*B - (Bp*y(3))*A - (C*B) + (D*A)) / (E*A) ;    %theta.
    dydt(4)= ((torque-Br*y(4)) - C + B * dydt(3))/A ;     %phi.
    dydt(5) = (Voltage(t,y) - R*y(5) - Ke*y(3)) / L; %current .


end

function dydt=D(t,y)
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
    dydt=odefun(t,y,mp,d,Ip,Bp,Br,l,Ir,g,R,L,Kt,Ke,Bm);
end


%simulation

y0=[(pi/6) 0 0 0 0];
t1=[0 100];
[t,y]=ode45(@D,t1,y0');


figure;
subplot(2,1,1);
plot(t, y(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(t, y(:,2), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angles (rad)');
legend('\theta (rod)', '\alpha (pendulum)');
title('Angular Positions');
grid on;

subplot(2,1,2);
plot(t, y(:,5), 'r--', 'LineWidth', 1.5); hold on;
xlabel("TIME");
ylabel("current");
grid on;

%% Updated Visualization (Add Energy Plot)
figure;
subplot(3,1,1);
plot(t, y(:,1), 'r', t, y(:,2), 'b');
legend('\alpha (pendulum)', '\theta (arm)');
title('Angular Positions');
grid on;

subplot(3,1,2);
plot(t, y(:,5), 'g');
ylabel('Current (A)');
title('Motor Current');
grid on;

subplot(3,1,3);
energy = 0.5*1*y(:,3).^2 + 1*9.81*0.5*(1 - cos(y(:,1))); % E = 0.5Ipω² + mgh
plot(t, energy, 'k');
ylabel('Energy (J)');
title('Pendulum Energy');
grid on;



%plot% 

L1 = 1;  % Length of arm
L2 = 0.5; % Length of pendulum

figure;
hold on;
grid on;
axis equal;
xlim([-L1 - L2, L1 + L2]);
ylim([-L1 - L2, L1 + L2]);
zlim([-L2, L1 + L2]);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Motion of the System');
view(3); % Ensure 3D perspective

% Initialize plot elements
line1 = plot3([0 0], [0 0], [0 0], 'bo-', 'LineWidth', 2); % Origin to A
line2 = plot3([0 0], [0 0], [0 0], 'ro-', 'LineWidth', 2); % A to B

for k = 1:length(t)
    % Compute point A
    Ax = L1 * cos(y(k, 2));
    Ay = L1 * sin(y(k, 2));
    Az = 0;

    % Compute point B
    Bx = Ax - L2 * sin(y(k, 1)) * sin(y(k, 2));
    By = Ay + L2 * sin(y(k, 1)) * cos(y(k, 2));
    Bz = (-1)*(Az - L2 * cos(y(k, 1)));

    % Update the lines dynamically
    set(line1, 'XData', [0 Ax], 'YData', [0 Ay], 'ZData', [0 Az]); % Origin to A
    set(line2, 'XData', [Ax Bx], 'YData', [Ay By], 'ZData', [Az Bz]); % A to B

    drawnow; % Update the figure
    pause(0.05); % Slow down animation
end

hold off;