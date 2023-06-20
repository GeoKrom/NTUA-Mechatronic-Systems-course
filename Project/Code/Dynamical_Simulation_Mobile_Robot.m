% This is a script file for dynamic modelling
% and simulation of a two wheel mobile robot
clear variables;
clc;

m = 0.8; % kg
I = .625;  % kg*m^2
v1 = 2.2;
v2 = 2.5;
c1 = 0.98;
c2 = 0.91;
% Initial State
t_start = 0; % sec, starts simulation time
t_end   = 20; % sec, ends simulation time
dt = 0.0001; % time step of the simulation
lin_vel = 0;
ang_vel = 0;
t_sim = t_start:dt:t_end; % simulation time steps
N = numel(t_sim);         % Number of points in the simulation
lin_vel_dot = 0;
ang_vel_dot = 0;

% Vector of torque ô = [Fs Ms]
% Fs = 2.5*sin(2*t_sim); % N
% Ms = .5*sin(2*t_sim);  % N*m
Fs = 0.2;
Ms = 0;
% Fs = zeros(1,N);
% Ms = zeros(1,N);
% for i = 1:N
%     if i < (N/2+1)
%         Fs(i) = 2.5;
%     else
%         Fs(i) = 0;
%     end
% end
% for i = 1:N
%     if i < (N/2+1)
%         Ms(i) = 1;
%     else
%         Ms(i) = 0;
%     end
% end
% Output values after the simulation
lin_vel_out = zeros(1, N); % robot linear velocity
ang_vel_out = zeros(1, N); % robot angular velocity
lin_vel_dot_out = zeros(1, N);
ang_vel_dot_out = zeros(1, N);
x_dot_out = zeros(1,N);
y_dot_out = zeros(1,N);
x_out = zeros(1,N);
y_out = zeros(1,N);
x = 0;
y = 0;
theta_dot_out = zeros(1,N);
theta_out = zeros(1,N);
lin_vel_desired = zeros(1,N);
ang_vel_desired = zeros(1,N);
theta = 0;
% SIMULATION PROFILE
index = 0;
for i = t_sim
    index = index + 1;
    
    % Kinematics
    theta_dot_out(index) = ang_vel;
    theta = theta + ang_vel.*dt;
    theta_out(index) = theta;
    x_dot_out(index) = lin_vel.*cos(theta);
    y_dot_out(index) = lin_vel.*sin(theta);
    x = x + x_dot_out(index).*dt;
    y = y + y_dot_out(index).*dt;
    x_out(index) = x;
    y_out(index) = y;

    lin_vel_dot = (Fs - v1*lin_vel+c1*sign(lin_vel))/m ;     % Linear_Accelaration
    ang_vel_dot = (Ms - v2*ang_vel+c2*sign(ang_vel))/I ;     % Angular_Accelaration
    
    lin_vel = lin_vel + lin_vel_dot*dt; % Linear_Velocity
    ang_vel = ang_vel + ang_vel_dot*dt; % Angular_Velocity

    lin_vel_dot_out(index) = lin_vel_dot;
    ang_vel_dot_out(index) = ang_vel_dot; 
    lin_vel_out(index) = lin_vel;   % save the linear velocity value
    ang_vel_out(index) = ang_vel;   % save the angular velocity value
end

% PLOTS
figure(1);
clf;
subplot(2,1,1);
plot(t_sim, lin_vel_dot_out,'r-');
grid;
ylabel("$\dot{v}  [m/sec^2]$ ", "Interpreter","latex","FontSize",16);
xlabel("time [sec]", "Interpreter","latex","FontSize",16);
subplot(2,1,2);
plot(t_sim, lin_vel_out,'r-');
grid;
ylabel('$v [m/sec]$',"Interpreter","latex","FontSize",16);
xlabel("time [sec]", "Interpreter","latex","FontSize",16);

figure(2);
clf;
subplot(2,1,1);
plot(t_sim, ang_vel_dot_out,'r-');
grid;
ylabel("$\dot{\omega}  [rad/sec^2]$ ", "Interpreter","latex","FontSize",16);
xlabel("time [sec]", "Interpreter","latex","FontSize",16);
subplot(2,1,2);
plot(t_sim, ang_vel_out,'r-');
grid;
ylabel('$\omega [rad/sec]$ ',"Interpreter","latex","FontSize",16);
xlabel("time [sec]", "Interpreter","latex","FontSize",16);

figure(3);
clf;
subplot(3,1,1);
plot(t_sim, x_dot_out, 'r-');
grid on;
xlabel("time [sec]", "Interpreter","latex","FontSize",16)
ylabel("$\dot{x}$ [m/sec]","Interpreter","latex","FontSize",16);

subplot(3,1,2);
plot(t_sim, y_dot_out, 'r-');
grid on;
xlabel("time [sec]", "Interpreter","latex","FontSize",16)
ylabel("$\dot{y}$ [m/sec]","Interpreter","latex","FontSize",16);

subplot(3,1,3);
plot(t_sim, theta_dot_out, 'r-');
grid on;
xlabel("time [sec]", "Interpreter","latex","FontSize",16)
ylabel("$\dot{\vartheta}$ [rad/sec]","Interpreter","latex","FontSize",16);

figure(4);
clf;
subplot(3,1,1);
plot(t_sim, x_out, 'r-');
grid on;
xlabel("time [sec]", "Interpreter","latex","FontSize",16)
ylabel("x [m]","Interpreter","latex","FontSize",16);

subplot(3,1,2);
plot(t_sim, y_out, 'r-');
grid on;
xlabel("time [sec]", "Interpreter","latex","FontSize",16)
ylabel("y [m]","Interpreter","latex","FontSize",16);

subplot(3,1,3);
plot(t_sim, theta_out, 'r-');
grid on;
xlabel("time [sec]", "Interpreter","latex","FontSize",16)
ylabel("$\vartheta$ [rad]","Interpreter","latex","FontSize",16);

figure(5);
clf;
plot(x_out, y_out, 'r-');
grid on;
xlabel("x [m]", "Interpreter","latex","FontSize",16);
ylabel("y [m]","Interpreter","latex","FontSize",16);
