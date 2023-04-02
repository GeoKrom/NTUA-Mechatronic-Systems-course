%% Modelling and simulation of thw pinion system of a CNC machine

clear;
clc;


%% Parameters of System

Bm = 0.03;                  % Nms/rad
K = 8500;                   % Nm/rad
Jm = 0.0075;                % Nms^2/rad
B1 = 15;                    % Ns/m
mc = 65;                    % kg
r = 0.1;                    % m
Jp = 0.0025;                % Nms^2/rad
kT = 1;                     % Nm/A
t = 0:0.0001:40;            % sec
is = 5;                     % A
%% System in State Space Form
A = [-Bm/Jm -1/Jm 0; K 0 -K/r; 0 1/(Jp/r + mc*r) -B1/(Jp/(r^2) + mc)];
B = [kT/Jm; 0; 0];
C = eye(3);                 % or [0 0 1] for only the output state;
D = 0;
u = 5*ones(size(t));
x0 = [0 0 0];
system = ss(A,B,C,D);
system.OutputName = {'w_J_m','T_K','v_m_c'};
stateVector = lsim(system,u,t,x0);

xss = -inv(A)*B*is;

disp(xss);
figure(1);
clf;
subplot(3,1,1);
plot(t,u,'r--');
hold on;
plot(t, stateVector(:,1),'b-');
xlabel("time [sec]");
ylabel("w_J_m [rad/sec]");
grid on;
legend('Input Current (A)', 'State Variable 1 (Angular Velocity)','Location','southeast');

subplot(3,1,2);
plot(t,u,'r--');
hold on;
plot(t, stateVector(:,2),'b-');
xlabel("time [sec]");
ylabel("T_K [Nm]");
grid on;
legend('Input Current (A)', 'State Variable 2 (Spring Torque)','Location','northeast');

subplot(3,1,3);
plot(t,u,'r--');
hold on;
plot(t, stateVector(:,3),'b-');
xlabel("time [sec]");
ylabel("v_m_c [m/sec]");
grid on;
legend('Input Current (A)', 'State Variable 3 (Carriage Linear Velocity)','Location','southeast');

%% System in analytical solution (Differential Equation)

c0 = -2.779002026809371;
c1 = 2.02680937099213e-06;
c2 = -0.000555988519621612;
v = c0*exp(-0.23214*t) + c1*exp(-1.9988*t).*cos(1160.3*t)+ c2*exp(-1.9988*t).*sin(1160.3*t) + 2.779;
yss = 2.779*ones(size(t));

figure(2);
clf;
plot(t,yss,'b--');
hold on;
grid on;
plot(t,v,'r-');
xlabel("t [sec]")
ylabel("v_m_c [m/sec]")
legend("y_s_s","Carrige Linear Velocity (ODE)","Location","southeast");

%% Compute the roots of the characteristic polynomial (or poles of the system)
p = [1 4.2298 1346361.0721 312547.28];
r = roots(p);
figure(3);
clf;
plot(real(r),imag(r),'kx','LineWidth', 2);
grid;
xlabel("Re","fontsize", 16);
ylabel("Im", "fontsize", 16);
legend('Poles of the system','Location','northeast','fontsize',14);
