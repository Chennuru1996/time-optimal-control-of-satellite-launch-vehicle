clc
clear all
% Lunar Rocket Soft Landing PD Controller with Plotting

% Constants and Initial Conditions
g = 1.62;           % Acceleration due to gravity on the lunar surface (m/s^2)
m = 2000;           % Mass of the rocket (kg)
k_p = 100;          % Proportional Gain
k_d = 50;           % Derivative Gain
x_des = 100;        % Desired landing location (m)
v_des = 0;          % Desired landing velocity (m/s)
x = 20000;            % Initial altitude (m)
v = -10;           % Initial velocity (m/s)
t = 0;              % Initial time (s)
dt = 0.01;          % Time step (s)

% Initialize arrays for plotting
time = [t];
altitude = [x];
velocity = [v];

% PD Controller Loop
while x > 0
    % Calculate error and control input
    e = x_des - x;
    ed = v_des - v;
    u = k_p*e + k_d*ed;
    
    % Apply control input and update state
    a = g + u/m;
    v = v + a*dt;
    x = x + v*dt;
    t = t + dt;
    
    % Append state to plotting arrays
    time = [time, t];
    altitude = [altitude, x];
    velocity = [velocity, v];
end

% Display final landing results
fprintf('\nLanding complete at t=%.2f s\n', t);
%fprintf('Landing location: %.2f m\n', x);
%fprintf('Landing velocity: %.2f m/s\n', v);

% Plot altitude and velocity over time
%figure;
%subplot(2,1,1);
plot(time, altitude);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Rocket Altitude over Time');

% subplot(2,1,2);
% plot(time, velocity);
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% title('Rocket Velocity over Time');
