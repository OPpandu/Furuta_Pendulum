
function tau=torque(t,y)
    tau=0;
end

%%differential equations

function dydt=odefun(t,y,mp,d,Ip,Bp,Br,l,Ir,g)
    
    A=(mp*l*l + mp*d*d*cos(y(1)) + Ir);
    B=mp*l*d*cos(y(1));
    C=(mp*y(3)*y(4)*d*d*sin(2*y(1))) + mp*d*l*sin(y(1))*y(3)*y(3);
    D=(mp*d*(d/2)*sin(2*(y(1)))) + mp*g*d*sin(y(1));
    E=mp*d*d+Ip;

    dydt=zeros(4,1);
    dydt(1)= y(3); %theta
    dydt(2)= y(4); %phi
    dydt(3)= ((torque(t,y) - Br*y(4))*B - (Bp*y(3))*A - (C*B) + (D*A)) / (E*A) ;    %theta.
    dydt(4)= ((torque(t,y)-Br*y(4)) - C + B * dydt(3))/A ;     %phi.

end

function dydt=D(t,y)
    %constants%
    mp=1; % mass of pendulum
    d=0.5; %distance of COM of pendulum from piviot
    Ip=1; %MOI of pendulum about COM;
    Bp=0; %damping coefficient of pendulum
    Br=0; %damping coefficient of arm
    l=1; %length of arm
    Ir=1; %MOI of arm wrt piviot
    g=9.81; %gravity
    dydt=odefun(t,y,mp,d,Ip,Bp,Br,l,Ir,g);
end


%simulation

y0=[pi/4 0 0 0];
t1=[0 10];
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
plot(t, y(:,3), 'r--', 'LineWidth', 1.5); hold on;
plot(t, y(:,4), 'b--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angular Velocities (rad/s)');
legend('\dot{\theta} (rod)', '\dot{\alpha} (pendulum)');
title('Angular Velocities');
grid on;




%plot% 

% 
% L1 = 1;  % Length of arm
% L2 = 0.5; % Length of pendulum
% 
% figure;
% hold on;
% grid on;
% axis equal;
% xlim([-L1 - L2, L1 + L2]);
% ylim([-L1 - L2, L1 + L2]);
% zlim([-L2, L1 + L2]);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('3D Motion of the System');
% view(3); % Ensure 3D perspective
% 
% % Initialize plot elements
% line1 = plot3([0 0], [0 0], [0 0], 'bo-', 'LineWidth', 2); % Origin to A
% line2 = plot3([0 0], [0 0], [0 0], 'ro-', 'LineWidth', 2); % A to B
% 
% for k = 1:length(t)
%     % Compute point A
%     Ax = L1 * cos(y(k, 2));
%     Ay = L1 * sin(y(k, 2));
%     Az = 0;
% 
%     % Compute point B
%     Bx = Ax - L2 * sin(y(k, 1)) * sin(y(k, 2));
%     By = Ay + L2 * sin(y(k, 1)) * cos(y(k, 2));
%     Bz = (-1)*(Az - L2 * cos(y(k, 1)));
% 
%     % Update the lines dynamically
%     set(line1, 'XData', [0 Ax], 'YData', [0 Ay], 'ZData', [0 Az]); % Origin to A
%     set(line2, 'XData', [Ax Bx], 'YData', [Ay By], 'ZData', [Az Bz]); % A to B
% 
%     drawnow; % Update the figure
%     pause(0.05); % Slow down animation
% end
% 
% hold off;