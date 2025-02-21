%% Script to show the experimental results
% This script is used to show the experimental results of the FLAir
% experiments. The data is loaded from the log files and plotted.
%
% Workspace cleaning. 
clearvars; close all; clc;
% Add path to the FLAir postprocessing functions
addpath('FLAir_postprocessing/src/')
settings_json = 'config/settings_showExperimentalResults.json';

% Load the data
FlAir_data = GetLoggedData(settings_json);
dataLog = FlAir_data.loadBatch;

%% Plot the data
close all; clc;

% Plot the position of the drone
figure;
subplot(2,1,1);
plot(dataLog.socket05.t, dataLog.socket05.x(:,1), 'b:', 'LineWidth', 2);
hold on;
plot(dataLog.socket05.t, dataLog.socket05.xd(:,1), 'k', 'LineWidth', 2);
%plot(dataLog.vrpn1.t, dataLog.vrpn1.x(:,1), 'r--', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('X [m]');
legend({'Measured', 'Desired'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'Interpreter', 'latex');

subplot(2,1,2);
plot(dataLog.socket05.t, dataLog.socket05.x(:,2), 'b:', 'LineWidth', 2);
hold on;
plot(dataLog.socket05.t, dataLog.socket05.xd(:,2), 'k', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Y [m]');

% Plot the XY plane trajectory
figure;
plot(dataLog.socket05.x(:,1), dataLog.socket05.x(:,2), 'b:', 'LineWidth', 2);
hold on;
plot(dataLog.socket05.xd(:,1), dataLog.socket05.xd(:,2), 'k', 'LineWidth', 2);
xlabel('X [m]');
ylabel('Y [m]');
legend({'Measured', 'Desired'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'Interpreter', 'latex');

% Show the desired path and real trajectory using colormaps.
step = 100;
dataLog.socket05.t_lf = reduceSampleRate(dataLog.socket05.t, step, true);
dataLog.socket05.x_lf = reduceSampleRate(dataLog.socket05.x, step, true);
dataLog.socket05.xd_lf = reduceSampleRate(dataLog.socket05.xd, step, true);

figure;
patch(dataLog.socket05.x_lf(:,1), dataLog.socket05.x_lf(:,2), dataLog.socket05.t_lf, 'EdgeColor', 'interp', 'FaceColor', 'none');
hold on
patch(dataLog.socket05.xd_lf(:,1), dataLog.socket05.xd_lf(:,2), dataLog.socket05.t_lf, 'EdgeColor', 'interp', 'FaceColor', 'none', 'LineStyle', ':');
colorbar;
xlabel('X [m]');
ylabel('Y [m]');
legend({'Measured', 'Desired'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'Interpreter', 'latex');

% Plot the tracking error
figure;
dataLog.socket05 = computeL2ErrorXY(dataLog.socket05);
plot(dataLog.socket05.t, dataLog.socket05.errorL2, 'b:', 'LineWidth', 2);
hold on;
yline(mean(dataLog.socket05.errorL2), 'k', 'LineWidth', 2);
legend({'Error', 'Mean error'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'Interpreter', 'latex');
xlabel('Time [s]');
ylabel('Error [m]');

function new_vector = reduceSampleRate(vector, step, addNaN)
    % Take every step-th element of the vector
    [r,~] = size(vector);
    i = 1:step:r;
    new_vector = vector(i,:); 
    if addNaN
        new_vector(end+1,:) = NaN;
    end
end

function dataLog_exp = computeL2ErrorXY(dataLog_exp)
    % Compute the L2 error
    dataLog_exp.errorL2 = ((dataLog_exp.xd(:,1) - dataLog_exp.x(:,1)).^2 + (dataLog_exp.xd(:,2) - dataLog_exp.x(:,2)).^2).^0.5;
end