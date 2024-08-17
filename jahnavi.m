% Constants and parameters
trials = 1; % Number of trials
incl_angle = pi/6; % Inclination angle in radians
g = 10; % Gravitational acceleration in m/s^2
mass_cart = 100; % Mass of the cart in kg

% PID controller constants
K_p = 300;
K_d = 300;
K_i = 10;

% Time setup
dt = 0.02;
t0 = 0;
t_end = 5;
t = t0:dt:t_end;

% Gravitational force on the cart (in the vertical direction)
F_g = -mass_cart * g; % [N] (Newtons)

% Preallocate arrays for storing results
displ_rail = zeros(trials, length(t)); % Displacement along the rail [m]
v_rail = zeros(trials, length(t)); % Velocity along the rail [m/s]
a_rail = zeros(trials, length(t)); % Acceleration along the rail [m/s^2]
pos_x_train = zeros(trials, length(t)); % Train's x-position [m]
pos_y_train = zeros(trials, length(t)); % Train's y-position [m]
e = zeros(trials, length(t)); % Proportional error [m]
e_dot = zeros(trials, length(t)); % Derivative of the error [m/s]
e_int = zeros(trials, length(t)); % Integral of the error [m*s]
pos_x_cube = zeros(trials, length(t)); % Cube's x-position [m]
pos_y_cube = zeros(trials, length(t)); % Cube's y-position [m]

% Tangential component of the gravitational force on the cart
F_ga_t =F_g * sin(incl_angle); % [N]

% Initial conditions for the train
init_pos_x = 120; % Initial horizontal position [m]
init_pos_y = 120 * tan(incl_angle) + 6.5; % Initial vertical position [m]
init_displ_rail = sqrt(init_pos_x^2 + init_pos_y^2); % Initial displacement along the rail [m]
init_vel_rail = 0; % Initial velocity along the rail [m/s]
init_a_rail = 0; % Initial acceleration along the rail [m/s^2]

% Number of trials
trials_magn = trials;
history = ones(1, trials); % Record the success/failure state of each trial

% Simulation loop for each trial
while trials > 0
    % Set the reference positions for the cube (falling object)
    [pos_x_cube_ref, pos_y_cube_ref] = set_x_ref(incl_angle); % Cube's initial positions
   
    
    % Determine the current trial index (adjust for MATLAB's 1-based indexing)
    times = trials_magn - trials + 1;
    
    % Assign initial positions of the cube for the current trial
    pos_x_cube(times, :) = pos_x_cube_ref;
    pos_y_cube(times, :) = pos_y_cube_ref - (g / 2) * t.^2; % Quadratic displacement due to gravity
    
    % Initialize win flag and delta for the trial
    win = false;
    delta = 1;
    k=1;
    
    % Implement PID for the train position
    for i = 1:length(t)
        % Insert the initial values into the beginning of the predefined arrays
        if i == 1
            displ_rail(times, 1) = init_displ_rail;
            pos_x_train(times, 1) = 120;
            pos_y_train(times, 1) = init_pos_y;
            v_rail(times, 1) = init_vel_rail;
            a_rail(times, 1) = init_a_rail;
            
        end
        
        % Compute the horizontal error
        e(times, i) = pos_x_cube_ref - pos_x_train(times, i);
        
        if i > 1
            % Calculate the derivative of the error (rate of change of error)
            e_dot(times, i) = (e(times, i) - e(times, i - 1)) / dt;
            
            % Calculate the integral of the error (accumulated error over time)
            e_int(times, i) = e_int(times, i - 1) + (e(times, i - 1) + e(times, i)) / 2 * dt;
        end
        
        if i == length(t)
            % Ensure the last error values are carried over if it's the last time step
            e(times, i) = e(times, i - 1);
            e_dot(times, i) = e_dot(times, i - 1);
            e_int(times, i) = e_int(times, i - 1);
        end
        
        % Calculate the control force using PID formula
        F_a = K_p * e(times, i) + K_d * e_dot(times, i) + K_i * e_int(times, i);
        
        % Calculate the net force on the train
        F_net = F_a+F_ga_t ;
        
        % Update the acceleration of the train
        a_rail(times, i) = F_net / mass_cart;
        
        if i > 1
            % Update the velocity of the train using trapezoidal integration
            v_rail(times, i) = v_rail(times, i - 1) + (a_rail(times, i - 1) + a_rail(times, i)) * dt/2;
            
            % Update the displacement along the rail
            displ_rail(times, i) = displ_rail(times, i - 1) + (v_rail(times, i - 1) + v_rail(times, i)) * dt/2;
        else
            % For the first time step, use initial values
            v_rail(times, i) = init_vel_rail;
            displ_rail(times, i) = init_displ_rail;
        end
        
        % Update the train's horizontal and vertical positions
        pos_x_train(times, i+1) = displ_rail(times, i) * cos(incl_angle);
        pos_y_train(times, i+1) = displ_rail(times, i) * sin(incl_angle) + 6.5;
        
        % Try to catch the cube
        if (pos_x_train(times, i+1) - 5 < pos_x_cube(times, i) + 3 && pos_x_train(times, i+1) + 5 > pos_x_cube(times, i) - 3) || win
            if (pos_y_train(times, i+1) + 3 < pos_y_cube(times, i) - 2 && pos_y_train(times, i+1) + 8 > pos_y_cube(times, i) + 2) || win
                % The cube is considered caught, set win flag to true
                win = true;
                p(k)=i;
                k=k+1;
                

                % Ensure that the cube moves with the train after being caught
                if delta == 1
                    change = pos_x_train(times, i+1) - pos_x_cube(times, i);
                    delta = 0;
                end
                pos_x_cube(times, i) = pos_x_train(times, i+1) - change;
                pos_y_cube(times, i) = pos_y_train(times, i+1)+5 ;
            end
        end
    end

    % Update initial conditions for the next trial based on the last state of the current trial
    init_displ_rail = displ_rail(times, end);
    init_pos_x = pos_x_train(times, end) + v_rail(times, end) * cos(incl_angle) * dt;
    init_pos_y = pos_y_train(times, end) + v_rail(times, end) * sin(incl_angle) * dt;
    init_vel_rail = v_rail(times, end);
    init_a_rail = a_rail(times, end);

    % Record the success/failure state for the current trial
    history(times) = delta;

    % Decrease the number of remaining trials
    trials = trials - 1;
end

% Function to generate random x-positions for a falling object
function [rand_h, rand_v] = set_x_ref(incl_angle)
    rand_h = rand * 120;
    rand_v = rand * (40 - 20) + 20 + 120 * tan(incl_angle) + 6.5;
end

% Analysis of Overshoot, Settling Time, and Rise Time
% Choose the final trial for analysis
output = pos_x_train(times, :); % Train's x-position over time
setpoint = pos_x_cube_ref; % Assume the final value is the setpoint for this analysis

% Overshoot
error = output-setpoint;
if(error(1)<0)
    overshoot_percentage = (max(error) / setpoint) * 100;
else
    overshoot_percentage = (-min(error) / setpoint) * 100;
end
% Settling Time (within 2% of setpoint)
tolerance =  0.6;
%settling_index = find(abs(output - setpoint) <= tolerance, 1, 'last');
settling_time = t(p(1));

% Rise Time (time to go from 10% to 90% of the final value)
rise_time_start = find(output >= 0.1 * setpoint, 1);
rise_time_end = find(output >= 0.9 * setpoint, 1);
rise_time = t(rise_time_end) - t(rise_time_start);

% Display results
fprintf('Overshoot: %.2f%%\n', overshoot_percentage);
fprintf('Settling Time: %.2f seconds\n', settling_time);
fprintf('Rise Time: %.2f seconds\n', rise_time);

% Plot the response with annotations
figure;
plot(t, output(1:end-1));
hold on;
plot([t(1), t(end)], [setpoint, setpoint], '--r'); % Setpoint line
title('System Response');
xlabel('Time (s)');
ylabel('Output');
grid on;
legend('System Response', 'Setpoint');
%text(settling_time, output(settling_index), sprintf('Settling Time: %.2f s', settling_time), 'VerticalAlignment', 'bottom');
%text(t(end), peak_value, sprintf('Overshoot: %.2f%%', overshoot_percentage), 'VerticalAlignment', 'top');

% Error Dynamics Visualization
figure;
subplot(3, 1, 1);
plot(t, e(times, :));
title('Proportional Error (e)');
xlabel('Time (s)');
ylabel('Error (m)');
grid on;

subplot(3, 1, 2);
plot(t, e_dot(times, :));
title('Derivative of Error (e\_dot)');
xlabel('Time (s)');
ylabel('Error Rate (m/s)');
grid on;

subplot(3, 1, 3);
plot(t, e_int(times, :));
title('Integral of Error (e\_int)');
xlabel('Time (s)');
ylabel('Accumulated Error (m*s)');
grid on;


