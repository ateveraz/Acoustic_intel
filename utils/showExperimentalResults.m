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
close all;

% Plot the position of the drone
figure;
subplot(2,1,1);
plot(dataLog.vrpn.t, dataLog.vrpn.x(:,1), 'b:', 'LineWidth', 2);
hold on;
plot(dataLog.vrpn.t, dataLog.vrpn.xd(:,1), 'k', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('X [m]');
legend({'Measured', 'Desired'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'Interpreter', 'latex');

subplot(2,1,2);
plot(dataLog.vrpn.t, dataLog.vrpn.x(:,2), 'b:', 'LineWidth', 2);
hold on;
plot(dataLog.vrpn.t, dataLog.vrpn.xd(:,2), 'k', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Y [m]');

% Plot the XY plane trajectory
figure;
plot(dataLog.vrpn.x(:,1), dataLog.vrpn.x(:,2), 'b:', 'LineWidth', 2);
hold on;
plot(dataLog.vrpn.xd(:,1), dataLog.vrpn.xd(:,2), 'k', 'LineWidth', 2);
xlabel('X [m]');
ylabel('Y [m]');
legend({'Measured', 'Desired'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'Interpreter', 'latex');

