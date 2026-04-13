% ============================================================
% STUDENT NAME: Paul Martin
% ME4640 Robot Homework B - Trajectory Verification Plot
%
% PURPOSE:
%   Simulates the programmed trajectory from RobotB_Martin_Trajectory.ino
%   and produces a plot of theta_1 through theta_4 vs. time.
%
% USAGE (two modes):
%   1. SIMULATION MODE (default):
%      Run this script directly. It replicates the Arduino trajectory
%      logic in MATLAB so you can verify the motion BEFORE uploading.
%
%   2. LIVE DATA MODE:
%      After running the Arduino code in verification mode (servos unpowered),
%      copy the CSV lines from Serial Monitor into a file, then set:
%          USE_SERIAL_CSV = true;
%          CSV_FILE = 'serial_output.csv';
% ============================================================

clear; close all; clc;

% ===== CONFIGURATION =====
USE_SERIAL_CSV = false;              % Set true to plot from captured Serial CSV data
CSV_FILE = 'serial_output.csv';      % CSV from Serial Monitor (time,th1,th2,th3,th4)

% ===== TRAJECTORY PARAMETERS (must match Arduino code) =====
DT = 0.010;       % simulation timestep (seconds) - matches LOOP_MS = 10 ms

% Target angles (degrees from home)
TH1_A    =  45.0;
TH2_A    =  40.0;
TH3_A    = -40.0;
TH4_OPEN =  35.0;

% Phase durations (seconds)
DURATIONS = [2.0, 1.0, 2.0, 1.0, 2.0, 1.0, 1.0, 1.0, 2.0];
PHASE_NAMES = {
    '1: Base 0→45°',
    '2: Hold Base',
    '3: Shoulder 0→40°',
    '4: Hold Shoulder',
    '5: Elbow 0→-40°',
    '6: Hold Elbow',
    '7: Gripper Open',
    '8: Gripper Close',
    '9: Return Base'
};

% ============================================================
% HELPER FUNCTION: Linear interpolation
% ============================================================
function val = lerp(a, b, t)
    t = max(0, min(1, t));  % Clamp t to [0,1]
    val = a + (b - a) * t;
end

% ============================================================
% SIMULATION FUNCTION
% ============================================================
function [times, th1_arr, th2_arr, th3_arr, th4_arr] = simulate_trajectory()
    % Access variables from base workspace
    DT = evalin('base', 'DT');
    TH1_A = evalin('base', 'TH1_A');
    TH2_A = evalin('base', 'TH2_A');
    TH3_A = evalin('base', 'TH3_A');
    TH4_OPEN = evalin('base', 'TH4_OPEN');
    DURATIONS = evalin('base', 'DURATIONS');
    
    times = [];
    th1_arr = [];
    th2_arr = [];
    th3_arr = [];
    th4_arr = [];
    
    th1 = 0.0;
    th2 = 0.0;
    th3 = 0.0;
    th4 = 0.0;
    t = 0.0;
    
    for phase_idx = 1:length(DURATIONS)
        duration = DURATIONS(phase_idx);
        
        % Store starting angles for this phase
        th1_start = th1;
        th2_start = th2;
        th3_start = th3;
        th4_start = th4;
        
        steps = round(duration / DT);
        
        for step = 0:steps
            if steps > 0
                t_frac = step / steps;
            else
                t_frac = 1.0;
            end
            
            % Update angles based on phase
            if phase_idx == 1       % Sweep th1: 0 → 45
                th1 = lerp(th1_start, TH1_A, t_frac);
            elseif phase_idx == 2   % Hold th1 at 45
                th1 = TH1_A;
            elseif phase_idx == 3   % Sweep th2: 0 → 40
                th2 = lerp(th2_start, TH2_A, t_frac);
            elseif phase_idx == 4   % Hold th2 at 40
                th2 = TH2_A;
            elseif phase_idx == 5   % Sweep th3: 0 → -40
                th3 = lerp(th3_start, TH3_A, t_frac);
            elseif phase_idx == 6   % Hold th3 at -40
                th3 = TH3_A;
            elseif phase_idx == 7   % Open gripper: 0 → 35
                th4 = lerp(th4_start, TH4_OPEN, t_frac);
            elseif phase_idx == 8   % Close gripper: 35 → 0
                th4 = lerp(th4_start, 0.0, t_frac);
            elseif phase_idx == 9   % Return th1: 45 → 0
                th1 = lerp(th1_start, 0.0, t_frac);
            end
            
            % Store values
            times = [times; t];
            th1_arr = [th1_arr; th1];
            th2_arr = [th2_arr; th2];
            th3_arr = [th3_arr; th3];
            th4_arr = [th4_arr; th4];
            
            t = t + DT;
        end
    end
end

% ============================================================
% LOAD SERIAL CSV FUNCTION
% ============================================================
function [times, th1, th2, th3, th4] = load_serial_csv(filepath)
    % Read CSV file, skipping header and comment lines
    fid = fopen(filepath, 'r');
    data = [];
    
    while ~feof(fid)
        line = fgetl(fid);
        
        % Skip comment lines, header lines, and separator lines
        if isempty(line) || line(1) == '#' || line(1) == '=' || line(1) == '-'
            continue;
        end
        if contains(lower(line), 'time_s')
            continue;
        end
        
        % Try to parse as CSV
        parts = strsplit(line, ',');
        if length(parts) == 5
            try
                row = str2double(parts);
                if ~any(isnan(row))
                    data = [data; row];
                end
            catch
                continue;
            end
        end
    end
    fclose(fid);
    
    times = data(:,1);
    th1 = data(:,2);
    th2 = data(:,3);
    th3 = data(:,4);
    th4 = data(:,5);
end

% ============================================================
% MAIN EXECUTION
% ============================================================
if USE_SERIAL_CSV
    fprintf('Loading Serial data from: %s\n', CSV_FILE);
    [t, th1, th2, th3, th4] = load_serial_csv(CSV_FILE);
    title_suffix = '(Live Serial Data)';
else
    fprintf('Running trajectory simulation...\n');
    [t, th1, th2, th3, th4] = simulate_trajectory();
    title_suffix = '(Simulated)';
end

% ===== COMPUTE PHASE BOUNDARIES =====
phase_starts = [0];
for i = 1:(length(DURATIONS)-1)
    phase_starts = [phase_starts, phase_starts(end) + DURATIONS(i)];
end
phase_starts = [phase_starts, phase_starts(end) + DURATIONS(end)];

% ===== CREATE PLOT =====
figure('Position', [100, 100, 1200, 600]);
hold on;
grid on;

% Joint angle lines
plot(t, th1, 'LineWidth', 2.0, 'Color', [0.12, 0.47, 0.71], 'DisplayName', '\theta_1 - Base');
plot(t, th2, 'LineWidth', 2.0, 'Color', [1.00, 0.50, 0.05], 'DisplayName', '\theta_2 - Shoulder');
plot(t, th3, 'LineWidth', 2.0, 'Color', [0.17, 0.63, 0.17], 'DisplayName', '\theta_3 - Elbow');
plot(t, th4, 'LineWidth', 2.0, 'Color', [0.84, 0.15, 0.16], 'DisplayName', '\theta_4 - Gripper');

% Phase shading (alternating backgrounds)
y_limits = [min(th3) - 8, max([max(th1), max(th2), 50]) + 12];
ylim(y_limits);

colors_bg = [0.94, 0.96, 1.00; 1.00, 0.97, 0.94];
for i = 1:(length(phase_starts)-1)
    t0 = phase_starts(i);
    t1 = phase_starts(i+1);
    
    % Shaded background
    fill([t0 t1 t1 t0], [y_limits(1) y_limits(1) y_limits(2) y_limits(2)], ...
         colors_bg(mod(i-1, 2)+1, :), 'EdgeColor', 'none', 'FaceAlpha', 0.25);
    
    % Phase boundary line
    plot([t0 t0], y_limits, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.7);
end

% Replot lines on top of shading
plot(t, th1, 'LineWidth', 2.0, 'Color', [0.12, 0.47, 0.71]);
plot(t, th2, 'LineWidth', 2.0, 'Color', [1.00, 0.50, 0.05]);
plot(t, th3, 'LineWidth', 2.0, 'Color', [0.17, 0.63, 0.17]);
plot(t, th4, 'LineWidth', 2.0, 'Color', [0.84, 0.15, 0.16]);

% Phase labels at top
y_label = y_limits(2) - 6;
for i = 1:(length(phase_starts)-1)
    t_mid = (phase_starts(i) + phase_starts(i+1)) / 2;
    text(t_mid, y_label, PHASE_NAMES{i}, ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
         'FontSize', 9, 'Color', [0.4 0.4 0.4], 'FontAngle', 'italic');
end

% Reference line at 0
plot([t(1) t(end)], [0 0], 'k-', 'LineWidth', 0.8);

% Formatting
xlabel('Time (s)', 'FontSize', 13);
ylabel('Joint Angle (degrees)', 'FontSize', 13);
title({['ME4640 HW-B — Programmed Joint Trajectory ' title_suffix], ...
       '\theta_1 - Base  |  \theta_2 - Shoulder  |  \theta_3 - Elbow  |  \theta_4 - Gripper'}, ...
       'FontSize', 13);
legend('Location', 'southeast', 'FontSize', 12);
set(gca, 'FontSize', 12);
grid on;
box on;

% ===== SAVE OUTPUTS =====
out_png = 'RobotB_Martin_TrajectoryPlot.png';
out_pdf = 'RobotB_Martin_TrajectoryPlot.pdf';

print(out_png, '-dpng', '-r200');
print(out_pdf, '-dpdf', '-bestfit');

fprintf('Saved: %s\n', out_png);
fprintf('Saved: %s\n', out_pdf);

fprintf('\nTrajectory verification plot complete!\n');
