%% Modelling and simulation of a moving libra system 
%% Name: Georgios Krommydas, A.M.: 02121208

clear;
clc;


%% Parameters of System

R = 4;                  % Ω
L = 0.001;              % H
m = 0.5;                % kg
K = 10000;              % N/m
B = 5;                  % Ns/m
kc = 5;                 % N/A
t = 0:0.0001:1;        % sec

%% State Space System 1

A1 = [-R/L   0    0; 
      0     0     K; 
     -kc/m -1/m  -B/m];

B1 = [1/L; 0; 0];

C1 = eye(3);   % or [0 0 1];
D1 = 0;
u = 10*ones(size(t));
x0 = [0 0 0];
system = ss(A1,B1,C1,D1);
system.OutputName = {'i_L','F_K','v_m'};
stateVector = lsim(system,u,t,x0);

figure(1);
clf;
subplot(3,1,1);
plot(t, stateVector(:,1),'r-');
xlabel("time [sec]");
ylabel("i_L [A]");
grid on;
legend('State Variable 1 (Current)','Location','southeast');

subplot(3,1,2);
plot(t, stateVector(:,2),'r-');
xlabel("time [sec]");
ylabel("F_K [N]");
grid on;
legend('State Variable 2 (Spring Force)','Location','northeast');

subplot(3,1,3);
plot(t, stateVector(:,3),'r-');
xlabel("time [sec]");
ylabel("v_m [m/sec]");
grid on;
legend('State Variable 3 (Libra Mass Velocity)','Location','southeast');

%% Poles of the system
p1 = [1 4010 60000 80000000];
r1 = roots(p1);

figure(2);
clf;
plot(real(r1),imag(r1),'kx','LineWidth', 2);
grid;
xlabel("Re","fontsize", 16);
ylabel("Im", "fontsize", 16);
legend('Poles of the system','Location','northeast','fontsize',14);

p2 = [4010 60000 80000000];
r2 = roots(p2);

figure(3);
clf;
plot(real(r2),imag(r2),'kx','LineWidth', 2);
grid;
xlabel("Re","fontsize", 16);
ylabel("Im", "fontsize", 16);
legend('Poles of the system','Location','northeast','fontsize',14);

s = tf('s');

sys1 = (-kc/m)*s/(4010*s^2 +60000*s + 80000000);

stepplot(sys1)
%% State Space 2

A2 = [0 K; -1/m -B/m];
B2 = [0; -kc/m];
C2 = eye(2);
D2 = 0;
u2 = 1*ones(size(t));
x1 = [0 0];
sys2 = ss(A2,B2,C2,D2);
sys2.OutputName = {'F_K','v_m'};
stateVector2 = lsim(sys2,u2,t,x1);

figure(5);
clf;
subplot(2,1,1);
plot(t, stateVector2(:,1),'r-');
xlabel("time [sec]");
ylabel("F_K [Nm]");
grid on;
legend('State Variable 1 (Spring Force)','Location','northeast');

subplot(2,1,2);
plot(t, stateVector2(:,2),'r-');
xlabel("time [sec]");
ylabel("v_m [m/sec]");
grid on;
legend('State Variable 2 (Libra Mass Velocity)','Location','northeast');


%% Poles of the system
p3 = [1 2.5 5000];
r3 = roots(p3);

figure(6);
clf;
plot(real(r3),imag(r3),'kx','LineWidth', 2);
grid;
xlabel("Re","fontsize", 16);
ylabel("Im", "fontsize", 16);
legend('Poles of the system','Location','northeast','fontsize',14);

