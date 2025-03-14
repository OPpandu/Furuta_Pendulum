% disp("HI")
% a=4;
% a=7;
% a;
% % this is a variable
% %{ 
% laa laaa laaa laaaa
% %}
% 
% x = 1+2i;
% 
% a= [1 2 4];
% b = [1 2 3];
% c = a*b';% " ' " mean transpose
% A = [1 4 9; 16 25 81; 64 125 243];
% A(2,3)%indexing starts from 1 
% B = [-1 -4 -9; -16 -25 81; 64 -125 243];
% 
% C = A*B;
% C = A.*B;% element wise multiplication
% A(1,:);%like numpy
% B(:,2);
% B';
% A = (1:10);%like np.linspace 
% B = 0:1:100;%like np.linspace
% 
% %disp(a);
% Z=zeros(3)
% Z=zeros(3,1);
% P=ones(1,4);
% I=eye(4);%identity matrix
% B = [1 2 3; 4 5 6; 7 8 9];
% A = [1 4 9; 16 25 36; 49 64 81];
% C=[A,B];
% C=[A;B];
% A_inv=inv(A);%inverse of matrix
% 
% x=linspace(0.001,10000,100);%like np.linespace(start,end,steps)
% y1=sin(x);
% y2=cos(x);
% %figure(1);
% %plot(x,y1,'b--','LineWidth',4);
% legend("sine");
% 
% %figure(2);
% %plot(x,y2,'g--','LineWidth',4);
% legend("cosine");
% 
% %figure(3);
% y3=log(x);
% %plot(x,y3,'LineWidth',4);
% 
% %figure(4);
% y4=log10(x);
% %plot(x,y4,'LineWidth',4);
% 
% %figure(5);
% y4=log10(x);
% %semilogx(x,y4,'LineWidth',4);
% % xlabel("X values");
% % xlabel("X values");
% % title("PLOT");
% % grid off %use gridoff
% %plot(x,y2,'r--');
% 
% x=0:0.8:2*pi;
% len=length(x);
% y=zeros(len,1);
% for i=1:len
%     y(i)=sin(x(i));
% end
% %plot(x,y);
% 
% x=0:0.1:2*pi;
% y1=sin(x);
% y2=cos(x);
% y3=exp(-x);
% y4=x.^2;
% 
% % subplot(2,2,1)
% % plot(x,y1,'r--');
% % title("sine")
% % 
% % 
% % subplot(2,2,2)
% % plot(x,y2,'b--');
% % title("cosine")
% % 
% % 
% % subplot(2,2,3);
% % plot(x,y3,'g--','LineWidth',4);
% % title("exp")
% % 
% % 
% % subplot(2,2,4);
% % plot(x,y4,'b--');
% % title("quadratic")
% 

%CONTROL SYSTEMS

syms s t;
y=exp(-t);
laplace(y);% as t is stimulating variable and we have declared 
          %as function of t it takes automatcally the varying variable t
          %and calculate the laplace transform by using integral e-s*t;!
y2=(1/(s+2))+1;
ilaplace(y2);
s=tf('s');
G = tf([1 3],[1 1]);
G=(s+3)/(s+1);
bodeplot(G);
step(G)

G1=(s+3)/(s+1);
G2=(s+3)/(s+1);
G_series = series(G1,G2);
G_parallel=parallel(G1,G2);
G_feedback=feedback(G1,G2,-1);


G10=1/s;
G11=1;
g1=parallel(G11,G10);
g2=2;
gnet1=series(g1,g2);
g3=1;
gnet2=parallel(gnet1,g3);
step(gnet2)

sys= tf([1 1],[1 2]);
sys1=zpk([],[-3 -4],12);
step(sys1);

stepinfo(sys1);
sys_2=tf([1],[1 3]);
%step(sys_2);

t=1:0.01:10;
u=t.*t;
lsim(sys_2,u,t)

stable_sys=tf([1],[1 3]);
unstable_sys=tf([1],[1 -3]);
marginal_sys=tf([1],[1 0 1]);
figure;
subplot(3,1,1);
step(stable_sys);
title("Stable System-poles in LHP");
grid on;

subplot(3,1,2);
step(marginal_sys,60);
title('marginally stable system, poles on img axis');
grid on;

subplot(3,1,3);
step(unstable_sys,7);
title('unstable system, poles on RHP');
grid on;

overdamped_sys=tf([3],[1 4 3]);

critically_damped_sys= tf([4],[1 4 4]);
underdamped_sys=tf([5],[1 2 5]);


step(underdamped_sys,"b",critically_damped_sys,'r',overdamped_sys,'g');

legend("Under_admped system","critically_damped_sys","overdamped_sys");
grid on;

bode(stable_sys)
nmp=tf([10 -2],[1 3]);
graph =  bodeplot(nmp);
setoptions(graph,'Grid','on','FreqsScale','log')

stable_sys=tf([1],[1 3]);
nyquist(stable_sys)
N=50;
sys= tf([1],[1 2 0]);
[mag, phase, w] = bode(sys);

% Convert Magnitude from dB to Linear
mag = squeeze(mag);  % Remove extra dimensions
phase = squeeze(phase);  % Convert phase to 1D array

% Convert Phase from Degrees to Radians
theta = deg2rad(phase);  

% Create Polar Plot
figure;
polarplot(theta, mag, 'r', 'LineWidth', 2); % Plot in red
title('Polar Plot from Bode Data');
rlim([0 1]);  % Fix radial limit to [0,5]
grid on;
