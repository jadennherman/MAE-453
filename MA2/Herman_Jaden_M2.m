%% MAE 453 
% MATLAB Assignment 2
% Jaden Herman

clear;
clc;
close all;

scriptFolder = fileparts(mfilename('fullpath'));
data = load(fullfile(scriptFolder, 'MA2.mat'));

%% Cases and init conds

ctrlCase = input('Input case 1-4:');

% Case 1
elev = 0;
ail = 0;
rud = 0;

if ctrlCase == 2
    elev = 5;
elseif ctrlCase == 3
    ail = 1;
elseif ctrlCase == 4
    rud = 1;
end

controls = deg2rad([elev; ail; rud]);
caseNames = {'No control deflection', 'Elevator = 5 deg', ...
    'Aileron = 1 deg', 'Rudder = 1 deg'};
caseLabel = sprintf('Case %d: %s', ctrlCase, caseNames{ctrlCase});

U1 = data.U1;
g = data.g;
Theta1 = 0;

data.X_Tu = 0;
data.M_Tu = 0;
data.M_Talpha = 0;
data.Z_alphadot = 0;
data.Z_q = 0;
data.Y_p = 0;
data.Y_r = 0;
data.N_Tbeta = 0;

%% Let E*xdot = F*x + G*controls
E = eye(9);
F = zeros(9);
G = zeros(9, 3);

% u_dot = -g*cos(Theta1)*theta + (X_u + X_Tu)*u + X_alpha*alpha + X_de*elev
F(1, [1 3 8]) = [data.X_u + data.X_Tu, data.X_alpha/U1, -g*cos(Theta1)];
G(1, 1) = data.X_de;

% v_dot = -U1*r + g*cos(Theta1)*phi + Y_beta*beta + Y_p*p + Y_r*r
%         + Y_da*ail + Y_dr*rud
F(2, [2 4 6 7]) = [data.Y_beta/U1, data.Y_p, data.Y_r - U1, g*cos(Theta1)];
G(2, [2 3]) = [data.Y_da, data.Y_dr];

% w_dot - Z_alphadot*alpha_dot = (U1 + Z_q)*q - g*sin(Theta1)*theta 
%                                + Z_u*u + Z_alpha*alpha + Z_de*elev
E(3, 3) = 1 - data.Z_alphadot/U1;
F(3, [1 3 5 8]) = [data.Z_u, data.Z_alpha/U1, U1 + data.Z_q, -g*sin(Theta1)];
G(3, 1) = data.Z_de;

% p_dot - (Ixz/Ixx)*r_dot = L_beta*beta + L_p*p + L_r*r
%                           + L_da*ail + L_dr*rud
E(4, 6) = -data.Ixz/data.Ixx;
F(4, [2 4 6]) = [data.L_beta/U1, data.L_p, data.L_r];
G(4, [2 3]) = [data.L_da, data.L_dr];

% q_dot - M_alphadot*alpha_dot = (M_u + M_Tu)*u + (M_alpha + M_Talpha)*alpha 
%                                + M_q*q + M_de*elev
E(5, 3) = -data.M_alphadot/U1;
F(5, [1 3 5]) = [data.M_u + data.M_Tu, (data.M_alpha + data.M_Talpha)/U1, data.M_q];
G(5, 1) = data.M_de;

% r_dot - (Ixz/Izz)*p_dot = (N_beta + N_Tbeta)*beta
%                             + N_p*p + N_r*r + N_da*ail + N_dr*rud
E(6, 4) = -data.Ixz/data.Izz;
F(6, [2 4 6]) = [(data.N_beta + data.N_Tbeta)/U1, data.N_p, data.N_r];
G(6, [2 3]) = [data.N_da, data.N_dr];

% Euler eqs: phi_dot = p, theta_dot = q, psi_dot = r
F(7, 4) = 1;
F(8, 5) = 1;
F(9, 6) = 1;

A = E\F;
B = E\G;

%% Integrate selected case
x0 = zeros(9, 1);
tspan = 0:0.1:120;    % seconds
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
[t, x] = ode45(@(t, x) A*x + B*controls, tspan, x0, options);

% Angles remain in radians during integration; convert only for plotting
anglesDeg = rad2deg(x(:, 7:9));
ratesDeg = rad2deg(x(:, 4:6));
fprintf('\n%s\n', caseLabel);
fprintf('All initial perturbations = 0. Simulation time = %.0f s.\n', t(end));

%% Plot translational velocity perturbations (Figure 1)
figure('Name', [caseLabel, ' - Translational velocities'], 'Color', 'w');
velocityLabels = {'u (ft/s)', 'v (ft/s)', 'w (ft/s)'};
for k = 1:3
    subplot(3, 1, k);
    plot(t, x(:, k), 'LineWidth', 1.4);
    grid on;
    xlabel('Time (s)');
    ylabel(velocityLabels{k});
end
sgtitle([caseLabel, ' | Translational velocity perturbations']);

%% Plot rotational velocity perturbations (Figure 2)
figure('Name', [caseLabel, ' - Rotational velocities'], 'Color', 'w');
rateLabels = {'p (deg/s)', 'q (deg/s)', 'r (deg/s)'};
for k = 1:3
    subplot(3, 1, k);
    plot(t, ratesDeg(:, k), 'LineWidth', 1.4);
    grid on;
    xlabel('Time (s)');
    ylabel(rateLabels{k});
end
sgtitle([caseLabel, ' | Rotational velocity perturbations']);

%% Plot Euler angle perturbations (Figure 3)
figure('Name', [caseLabel, ' - Euler angles'], 'Color', 'w');
angleLabels = {'\phi (deg)', '\theta (deg)', '\psi (deg)'};
for k = 1:3
    subplot(3, 1, k);
    plot(t, anglesDeg(:, k), 'LineWidth', 1.4);
    grid on;
    xlabel('Time (s)');
    ylabel(angleLabels{k});
end
sgtitle([caseLabel, ' | Euler angle perturbations']);