clear all;
[t, k] = mainCalc();  % Run main simulation
Q = k(1:30, 1:51);     % Position
DQ = k(1:30, 52:102);  % Velocity
D2Q = k(1:30, 103:153);% Acceleration


points = {
    'A', 1, [0, 0];                % Point A, Body 1 (reference point for Body 1)
    'B', 1, [0.3, 0.6];            % Point B, Body 1
    'D', 6, [0, 0];                % Point D, Body 6 (reference point for Body 6)
    'E', 6, [0.3, 0.3];            % Point E, Body 6
    'F', 3, [0.3, 0.4];            % Point F, Body 3
    'G', 9, [0, 0];                % Point G, Body 9 (reference point for Body 9)
    'H', 10, [0.1, -0.25];         % Point H, Body 10
    'I', 3, [-0.2, -0.4];          % Point I, Body 3 
    'J', 5, [0.35, 0.05];          % Point J, Body 5
    'K', 1, [-0.2, 0.4];           % Point K, Body 1
    'L', 3, [0, -0.1];             % Point L, Body 3
    'M', 2, [-0.2, -0.15];         % Point M, Body 2
    'N', 8, [0.05, -0.2];          % Point N, Body 8
};


% Prompt user to input a point name
point_name = input('Enter the name of the point to plot (e.g., A, G, M, etc.): ', 's');

% Find the selected point in the points array
selected_point = find(strcmpi(point_name, points(:, 1)));

if isempty(selected_point)
    error('Point not recognized. Please enter a valid point name from the system.');
else
    % Extract data for the selected point
    body_index = points{selected_point, 2};
    offset = points{selected_point, 3};

    % Extract position, velocity, and acceleration for the point
    i = body_index - 1;
    Px = Q(3 * i + 1, :) + offset(1);  % X-position
    Py = Q(3 * i + 2, :) + offset(2);  % Y-position
    PHIp = Q(3 * i + 3, :);            % Angular position (if needed)

    Vx = DQ(3 * i + 1, :);  % X-velocity
    Vy = DQ(3 * i + 2, :);  % Y-velocity
    PHIv = DQ(3 * i + 3, :);% Angular velocity (if needed)

    Ax = D2Q(3 * i + 1, :); % X-acceleration
    Ay = D2Q(3 * i + 2, :); % Y-acceleration
    PHIa = D2Q(3 * i + 3, :);% Angular acceleration (if needed)

    % Plot Position, Velocity, Acceleration for the selected point
    figure;
    subplot(3, 1, 1);
    plot(t, Px, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Position X [m]');
    title(['Position X for Point ', point_name]);

    subplot(3, 1, 2);
    plot(t, Vx, 'b', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Velocity X [m/s]');
    title(['Velocity X for Point ', point_name]);

    subplot(3, 1, 3);
    plot(t, Ax, 'g', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Acceleration X [m/s^2]');
    title(['Acceleration X for Point ', point_name]);

    figure;
    subplot(3, 1, 1);
    plot(t, Py, 'k', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Position Y [m]');
    title(['Position Y for Point ', point_name]);

    subplot(3, 1, 2);
    plot(t, Vy, 'm', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Velocity Y [m/s]');
    title(['Velocity Y for Point ', point_name]);

    subplot(3, 1, 3);
    plot(t, Ay, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Acceleration Y [m/s^2]');
    title(['Acceleration Y for Point ', point_name]);
end
