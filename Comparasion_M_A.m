% Load ADAMS data from Excel
adams_data = readtable('Displacement_X_Y.xlsx');

disp(head(adams_data)); 

% Extract data for comparison
time_adams = adams_data{:, 1};
x_adams = adams_data{:, 2} / 1000; % Convert X-displacement from mm to m
y_adams = adams_data{:, 3} / 1000; % Convert Y-displacement from mm to m

% Load MATLAB simulation data
[t, k] = mainCalc();  
Q = k(1:30, 1:51);     % Position matrix

% Select the point for comparison (e.g., Point A)
body_index = 1;  %  Point A
i = body_index - 1;
x_matlab = Q(3 * i + 1, :); % X-position from MATLAB
y_matlab = Q(3 * i + 2, :); % Y-position from MATLAB

% Resample MATLAB data to match ADAMS time points
x_matlab_interp = interp1(t, x_matlab, time_adams, 'linear');
y_matlab_interp = interp1(t, y_matlab, time_adams, 'linear');

% Plot comparison
figure;
subplot(2, 1, 1);
plot(time_adams, x_adams, 'r', 'LineWidth', 1.5, 'DisplayName', 'ADAMS X');
hold on;
plot(time_adams, y_matlab_interp, 'b--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB X');
xlabel('Time [s]');
ylabel('X-Displacement [m]');
title('Comparison of X-Displacement');
legend('show');
grid on;

subplot(2, 1, 2);
plot(time_adams, y_adams, 'r', 'LineWidth', 1.5, 'DisplayName', 'ADAMS Y');
hold on;
plot(time_adams, x_matlab_interp, 'b--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB Y');
xlabel('Time [s]');
ylabel('Y-Displacement [m]');
title('Comparison of Y-Displacement');
legend('show');
grid on;

% Calculate differences
x_diff = abs(x_adams - x_matlab_interp);
y_diff = abs(y_adams - y_matlab_interp);

% Display max differences
fprintf('Maximum difference in X: %.6f m\n', max(x_diff));
fprintf('Maximum difference in Y: %.6f m\n', max(y_diff));

%% For G

% Load ADAMS data from Excel
adams_data = readtable('DisplacementG_X_Y.xlsx');

% Display the first few rows to verify the structure
disp(head(adams_data)); 

% Extract data for comparison
time_adams = adams_data{:, 1}; 
x_adams = adams_data{:, 2} / 1000; % Convert X-displacement from mm to m
y_adams = adams_data{:, 3} / 1000; % Convert Y-displacement from mm to m

% Load MATLAB simulation data
[t, k] = mainCalc();  
Q = k(1:30, 1:51);     % Position matrix

% Debugging: Check positions for all bodies
disp('Checking all body positions at initial time...');
for test_index = 1:10 % Adjust range to include relevant bodies
    x_pos = Q(3 * (test_index - 1) + 1, 1); % X-coordinate
    y_pos = Q(3 * (test_index - 1) + 2, 1); % Y-coordinate
    fprintf('Body %d: X = %.4f, Y = %.4f\n', test_index, x_pos, y_pos);
end

% Select the point G for comparison
% Set body_index based on debug output where G's initial position matches
body_index = input('Enter the body index associated with Point G: ');
i = body_index - 1;

% Extract position, velocity, and acceleration for G
x_matlab = Q(3 * i + 1, :); % X-position from MATLAB
y_matlab = Q(3 * i + 2, :); % Y-position from MATLAB

% Resample MATLAB data to match ADAMS time points (if needed)
x_matlab_interp = interp1(t, x_matlab, time_adams, 'linear');
y_matlab_interp = interp1(t, y_matlab, time_adams, 'linear');

% Plot comparison
figure;
subplot(2, 1, 1);
plot(time_adams, x_adams, 'r', 'LineWidth', 1.5, 'DisplayName', 'ADAMS X');
hold on;
plot(time_adams, x_matlab_interp, 'b--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB X');
xlabel('Time [s]');
ylabel('X-Displacement [m]');
title('Comparison of X-Displacement for Point G');
legend('show');
grid on;

subplot(2, 1, 2);
plot(time_adams, y_adams, 'r', 'LineWidth', 1.5, 'DisplayName', 'ADAMS Y');
hold on;
plot(time_adams, y_matlab_interp, 'b--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB Y');
xlabel('Time [s]');
ylabel('Y-Displacement [m]');
title('Comparison of Y-Displacement for Point G');
legend('show');
grid on;

% Calculate differences
x_diff = abs(x_adams - x_matlab_interp);
y_diff = abs(y_adams - y_matlab_interp);

% Display max differences
fprintf('Maximum difference in X for Point G: %.6f m\n', max(x_diff));
fprintf('Maximum difference in Y for Point G: %.6f m\n', max(y_diff));
