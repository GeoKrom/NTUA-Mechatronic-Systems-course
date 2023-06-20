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

% Vector of torque τ = [Fs Ms]
% Fs = 2.5*sin(2*t_sim); % N
% Ms = .5*sin(2*t_sim);  % N*m
% Fs = 1;
% Ms = 0.5;
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
p = zeros(1,N);
for i = 1:N
    if i < (N/2+1)
        lin_vel_desired(i) = .5;
    else
        lin_vel_desired(i) = 0;
    end
end
for i = 1:N
    if i < (N/2+1)
        ang_vel_desired(i) = 1;
    else
        ang_vel_desired(i) = 0;
    end
end

lin_vel_dot_desired = 0;
ang_vel_dot_deired = 0;
Kp1 = 5;
Kp2 = 4;
Ki1 = 5;
Ki2 = 3.5;
Kd1 = .615;
Kd2 = .425;
int1_error = 0;
int2_error = 0;
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
    p(index) = sqrt(x.^2 + y.^2);
    % PID Controller
    error_lin_vel_dot = lin_vel_dot_desired - lin_vel_dot;
    error_ang_vel_dot = ang_vel_dot_deired - ang_vel_dot;
    error_lin_vel = lin_vel_desired(index) - lin_vel;
    error_ang_vel = ang_vel_desired(index) - ang_vel;
    integral1 = int1_error + error_lin_vel.*dt;
    integral2 = int2_error + error_ang_vel.*dt;
    
    u1 = Kp1*error_lin_vel + Ki1*integral1 + Kd1.*error_lin_vel_dot;
    u2 = Kp2*error_ang_vel + Ki2*integral2 + Kd2.*error_ang_vel_dot;
    
    % Dynamics
    lin_vel_dot = (u1 - v1*lin_vel-c1*sign(lin_vel))/m ;     % Linear_Accelaration
    ang_vel_dot = (u2 - v2*ang_vel-c2*sign(ang_vel))/I ;     % Angular_Accelaration
    
    lin_vel = lin_vel + lin_vel_dot.*dt; % Linear_Velocity
    ang_vel = ang_vel + ang_vel_dot.*dt; % Angular_Velocity
    
    int1_error = integral1;
    int2_error = integral2;
    

    % Store Values
    lin_vel_dot_out(index) = lin_vel_dot;
    ang_vel_dot_out(index) = ang_vel_dot; 
    lin_vel_out(index) = lin_vel;   % save the linear velocity value
    ang_vel_out(index) = ang_vel;   % save the angular velocity value

end

% PLOTS
figure(1);
clf;
subplot(2,1,1);
plot(t_sim, lin_vel_dot_out,"r-");
grid;
ylabel("$\dot{v}  [m/sec^2]$ ", "Interpreter","latex","FontSize",16);
xlabel("time [sec]", "Interpreter","latex","FontSize",16);
subplot(2,1,2);
plot(t_sim, lin_vel_out,"r-");
hold on;
plot(t_sim, lin_vel_desired, 'b--');
grid;
ylabel('$v [m/sec]$',"Interpreter","latex","FontSize",16);
xlabel("time [sec]", "Interpreter","latex","FontSize",16);

figure(2);
clf;
subplot(2,1,1);
plot(t_sim, ang_vel_dot_out,"r-");
grid;
ylabel("$\dot{\omega}  [rad/sec^2]$ ", "Interpreter","latex","FontSize",16);
xlabel("time [sec]", "Interpreter","latex","FontSize",16);
subplot(2,1,2);
plot(t_sim, ang_vel_out,"r-");
hold on;
plot(t_sim, ang_vel_desired, 'b--');
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
