clc; clear; close all;

square = [1 1 2 2 1;
          1 2 2 1 1;
          0 0 0 0 0];

figure(1)
grid on
view(3)
hold on
xlabel('x')
ylabel('y')
zlabel('z')

% --- Rotation 1: about z-axis (blue) ---
w1 = [0 0 1]';
for th = 10:10:360
    R = expr_PCM(w1, deg2rad(th));
    sqR = R * square;
    plot3(sqR(1,:), sqR(2,:), sqR(3,:), 'b-')
end

% --- Rotation 2: about y = -x axis (green) ---
w2 = [1 -1 0]';
w2 = w2/norm(w2);
for th = 10:10:360
    R = expr_PCM(w2, deg2rad(th));
    sqR = R * square;
    plot3(sqR(1,:), sqR(2,:), sqR(3,:), 'g-')
end

% --- Rotation 3: custom axis (red) ---
w3 = [1 1 1]';
w3 = w3 / norm(w3);
for th = 10:10:360
    R = expr_PCM(w3, deg2rad(th));
    sqR = R * square;
    plot3(sqR(1,:), sqR(2,:), sqR(3,:), 'r-')
end