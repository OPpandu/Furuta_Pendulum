% DIFFERENTIAL EQUATIONS

function tau=F(t)
    tau=0;
end

%calculation of the angular acc of pendulum
function dydt=theta_derivative(y,t,m,M,d,g,i1)
    x = F(t)*m*d*cos(y(1)) + ((m*d*y(2))^2)*cos(y(1))*sin(y(1)) - (M+m)*g*m*d*sin(y(1));
    z=(m*d*cos(y(1))^2)-(M+m)*i1;
    dydt=x/z;
end

%calculation of acc of the base
function dydt=x_derivative(y,t,m,M,d,g,i1)
 
    x=F(t)+m*d*(y(2)^2)*sin(y(1))-g*((m*d)^2)*sin(y(1))*cos(y(1))/i1;
    z=M+m-((m*d*cos(y(1)))/i1);
    dydt=x/z;
end

function dydt= odefn(t,y,m,M,d,g,i1)
    dydt=zeros(4,1);
    dydt(1) = y(2);
    dydt(3) = y(4);
    dydt(2) = theta_derivative(y,t,m,M,d,g,i1);
    dydt(4) = x_derivative(y,t,m,M,d,g,i1);
    %dydt(4) = 0;
end

function dydt=D(t,y)
    m=6.969;          %mass of the pendulum
    l=1;            %length of the pendulum
    i1=m*(l^2)/3;   %moi of the pendulum
    d=l/2;          %distance of the com of pendulum from the base
    M=.9;          %mass of the cart
    g=9.82;    %gravity,pi    
    dydt=odefn(t,y,m,M,d,g,i1);
end



y0=[pi/6 0 0 0];
t1=[0 10];
[t,y]=ode45(@D,t1,y0');

cart_l = 1; 
cart_h = 0.75;  
pendulum_l = 3; 
pendulum_length = 5;

figure;
axis([-5 5 -5 5]); 
axis manual;
grid
hold on;

x = y(1,3); 
cart = rectangle('Position', [x - cart_l/2, 0, cart_l, cart_h], 'FaceColor', 'blue');


theta = y(1,1);
pendulum_x = [x, x + pendulum_l * sin(theta)];
pendulum_y = [cart_h/2, cart_h/2 + pendulum_l * cos(theta)];
pendulum = line(pendulum_x, pendulum_y, 'Color', 'red', 'LineWidth', 5);

for k = 1:length(t)
    x = y(k,3);
    set(cart, 'Position', [x - cart_l/2, 0, cart_l, cart_h]);

    theta = y(k,1); 
    pendulum_x = [x, x + pendulum_l * sin(theta)];
    pendulum_y = [cart_h/2, cart_h/2 + pendulum_l * cos(theta)];
    set(pendulum, 'XData', pendulum_x, 'YData', pendulum_y);
    
    pause(0.05); 
end

hold off;
