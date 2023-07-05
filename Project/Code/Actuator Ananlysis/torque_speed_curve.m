% Script for Actuator T-ω Characteristic equation

Ra = 30;                % Ω
Kt =  784.532;          % mNm/A
n_sync = 125;           % rpm  
V = 3;                  % V

T_max = 78.4532;
w_max = 125;
w = linspace(0,125);
T = T_max-(T_max/w_max).*w;

% Plot the torque-speed curve
plot(w,T,'r');
xlabel('$\omega$ [rpm]','Interpreter',  'latex');
ylabel('$\tau$ [mNm]','Interpreter',  'latex');
title ('DC Motor Torque-Speed Characteristic','Interpreter',  'latex');
grid on;