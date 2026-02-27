close all; clc;

% --- Servo 1 ---
data1 = [
  580,   0,    0;
  780,  24,   23;
  980,  43,   43;
 1180,  61,   60;
 1380,  76,   74;
 1580,  90,   90;
 1780, 109,  106;
 1980, 125,  124;
 2180, 144,  142;
 2380, 165,  165
];

% --- Servo 2 ---
data2 = [
  473,    0.0,    0.0;
  613,   12.6,   12.6;
  753,   25.2,   25.2;
  893,   37.8,   37.8;
 1033,   50.4,   50.4;
 1173,   63.0,   63.0;
 1313,   75.6,   75.6;
 1453,   88.2,   88.2;
 1593,  100.8,  100.8;
 1733,  113.4,  113.4;
 1873,  126.0,  126.0;
 2013,  138.6,  138.6;
 2153,  151.2,  151.2;
 2293,  163.8,  163.8;
 2473,  180.0,  180.0
];

% --- Servo 3 ---
data3 = [
  553,    0,    0;
  693,   13,   13;
  833,   26,   26;
  973,   39,   39;
 1113,   52,   52;
 1253,   65,   65;
 1393,   78,   78;
 1533,   90,   90;
 1673,  103,  103;
 1813,  116,  116;
 1953,  129,  129;
 2093,  142,  142;
 2233,  155,  155;
 2373,  168,  168;
 2500,  180,  180
];

% --- Servo 4 ---
data4 = [
    0,      0.0,    0.0;
  155,     12.8,   12.8;
  310,     25.6,   25.6;
  465,     38.4,   38.4;
  620,     51.2,   51.2;
  775,     64.0,   64.0;
  930,     76.8,   76.8;
 1133,     90.0,   90.0;
 1285,    102.8,  102.8;
 1440,    115.6,  115.6;
 1595,    128.4,  128.4;
 1750,    141.2,  141.2;
 1905,    154.0,  154.0;
 2060,    166.8,  166.8;
 2173,    180.0,  180.0
];

allData = {data1, data2, data3, data4};

% Styling per assignment
colors  = {'r','g','b','k'};   % servo 1/2/3/4 colors
markers = {'+','o','*','x'};   % servo 1/2/3/4 markers
names   = {'Servo 1','Servo 2','Servo 3','Servo 4'};

% Fit order
fitorder = 1;

% -------------------- Plot Setup --------------------
figure(1); clf;
hold on; grid on;

title('Calibration Plot: PWM Command vs Angle');
xlabel('Servomotor Angle (deg)');
ylabel('Servomotor PWM command (microseconds)');
set(gca, 'FontSize', 12);

% -------------------- Plot + Fit Each Servo --------------------
legendHandles = gobjects(4,1);  % store one handle per servo

for i = 1:4
    data = allData{i};

    % Combine up + down into one set for fitting
    angles = [data(:,2); data(:,3)];
    us     = [data(:,1); data(:,1)];
    valid  = ~isnan(angles);
    angles = angles(valid);
    us     = us(valid);

    % Fit: microseconds = f(angle)
    p = polyfit(angles, us, fitorder);

    % Plot raw points (markers only)
    plot(data(:,2), data(:,1), [colors{i} markers{i}], ...
        'MarkerSize', 8, 'LineWidth', 1.8);
    plot(data(:,3), data(:,1), [colors{i} markers{i}], ...
        'MarkerSize', 8, 'LineWidth', 1.8);

    % Plot fitted curve (SAVE HANDLE FOR LEGEND)
    thetaFit = linspace(min(angles), max(angles), 250);
    usFit    = polyval(p, thetaFit);

    legendHandles(i) = plot(thetaFit, usFit, ...
        [colors{i} '-'], 'LineWidth', 2);

    % Equation text
    if fitorder == 1
        eqn = sprintf('%s: us = %.4f*\\theta + %.2f', names{i}, p(1), p(2));
    else
        eqn = sprintf('%s: us = %.6f*\\theta^2 + %.4f*\\theta + %.2f', ...
                      names{i}, p(1), p(2), p(3));
    end

    text(0.02, 0.98 - 0.06*(i-1), eqn, ...
        'Units','normalized', ...
        'Color', colors{i}, ...
        'FontSize', 11, ...
        'BackgroundColor','w');
end

% Proper legend (one entry per servo)
legend(legendHandles, names, 'Location', 'best');


% -------------------- Save Figure --------------------
saveas(gcf, 'Robota_Martin_CalibrationPlot.png');
print(gcf, 'Robota_Martin_CalibrationPlot', '-dpdf', '-bestfit');
