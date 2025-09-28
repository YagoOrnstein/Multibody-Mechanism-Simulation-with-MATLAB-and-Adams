clear all;
[t, k] = mainCalc();  % Run main simulation
Q = k(1:30, 1:51);     % Position
DQ = k(1:30, 52:102);  % Velocity
D2Q = k(1:30, 103:153);% Acceleration

% Define center of mass (CofM) for each body
CofM = {
    'Body1', 1;  % Body 1's center of mass, associated with generalized coordinates 1
    'Body2', 2;  % Body 2's center of mass
    'Body3', 3;  % Body 3's center of mass
    'Body7', 7;  % Body 7's center of mass
    'Body9', 9;  % Body 9's center of mass
};

% Prompt user to input the body name
body_name = input('Enter the name of the body (e.g., Body1, Body2, Body3): ', 's');

% Find the selected body in the CofM array
selected_body = find(strcmpi(body_name, CofM(:, 1)));

if isempty(selected_body)
    error('Body not recognized. Please enter a valid body name.');
else
    % Extract data for the selected body's center of mass
    body_index = CofM{selected_body, 2};

    % Extract position, velocity, and acceleration for the body's CofM
    i = body_index - 1;
    Px = Q(3 * i + 1, :);  % X-position of the CofM
    Py = Q(3 * i + 2, :);  % Y-position of the CofM
    PHIp = Q(3 * i + 3, :);% Angular position (if needed)

    Vx = DQ(3 * i + 1, :);  % X-velocity of the CofM
    Vy = DQ(3 * i + 2, :);  % Y-velocity of the CofM
    PHIv = DQ(3 * i + 3, :);% Angular velocity (if needed)

    Ax = D2Q(3 * i + 1, :); % X-acceleration of the CofM
    Ay = D2Q(3 * i + 2, :); % Y-acceleration of the CofM
    PHIa = D2Q(3 * i + 3, :);% Angular acceleration (if needed)

    % Plot Position, Velocity, Acceleration for the body's CofM
    figure;
    subplot(3, 1, 1);
    plot(t, Px, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Position X [m]');
    title(['Position X for ', body_name, ' Center of Mass']);

    subplot(3, 1, 2);
    plot(t, Vx, 'b', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Velocity X [m/s]');
    title(['Velocity X for ', body_name, ' Center of Mass']);

    subplot(3, 1, 3);
    plot(t, Ax, 'g', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Acceleration X [m/s^2]');
    title(['Acceleration X for ', body_name, ' Center of Mass']);

    figure;
    subplot(3, 1, 1);
    plot(t, Py, 'k', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Position Y [m]');
    title(['Position Y for ', body_name, ' Center of Mass']);

    subplot(3, 1, 2);
    plot(t, Vy, 'm', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Velocity Y [m/s]');
    title(['Velocity Y for ', body_name, ' Center of Mass']);

    subplot(3, 1, 3);
    plot(t, Ay, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Acceleration Y [m/s^2]');
    title(['Acceleration Y for ', body_name, ' Center of Mass']);
end
