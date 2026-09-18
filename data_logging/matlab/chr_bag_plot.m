function chr_bag_plot(csv_file)
%CHR_BAG_PLOT Plot flight, arm, and whole-body CHR diagnostics.
% Run without an argument to select a CSV exported by bag_to_csv.py.

close all;
clc;

if nargin < 1 || strlength(string(csv_file)) == 0
    [file, folder] = uigetfile('*.csv', 'Select a CHR bag CSV');
    if isequal(file, 0)
        return;
    end
    csv_file = fullfile(folder, file);
end

data = readtable(csv_file, 'VariableNamingRule', 'preserve');
required = {'time_s', 'topic'};
if ~all(ismember(required, data.Properties.VariableNames))
    error('CHR:InvalidCSV', 'CSV must contain time_s and topic columns.');
end

plot_palletrone(data, csv_file);
plot_arm(data, csv_file);
plot_chr(data, csv_file);
end


function plot_palletrone(data, csv_file)
state = topic_mask(data, '/chr/state');
reference = topic_mask(data, '/chr/reference');
flight = topic_mask(data, '/chr/diagnostics/flight');

figure('Name', 'CHR - Palletrone flight control', 'Color', 'w', ...
    'Position', [40, 40, 1700, 1000]);
layout = tiledlayout(5, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, "Palletrone flight control | " + string(csv_file), ...
    'Interpreter', 'none');

nexttile;
plot_vector_pair(data, state, 'base_pose.position', reference, ...
    'base_pose.position', {'x', 'y', 'z'}, 1.0);
axis_label('Position [m]', 'meas', 'des');

nexttile;
plot_tracking_error(data, state, 'base_pose.position', reference, ...
    'base_pose.position', {'ex', 'ey', 'ez'}, 1.0, false);
axis_label('Position error [m]');

nexttile;
plot_vector_pair(data, state, 'base_twist.linear', reference, ...
    'base_twist.linear', {'vx', 'vy', 'vz'}, 1.0);
axis_label('Linear velocity [m/s]', 'meas', 'des');

nexttile;
plot_attitude_pair(data, state, 'base_pose.orientation', reference, ...
    'base_pose.orientation');
axis_label('Attitude [deg]', 'meas', 'des');

nexttile;
plot_attitude_error(data, state, 'base_pose.orientation', reference, ...
    'base_pose.orientation');
axis_label('Attitude error [deg]');

nexttile;
plot_vector_pair(data, state, 'base_twist.angular', reference, ...
    'base_twist.angular', {'wx', 'wy', 'wz'}, 1.0);
axis_label('Angular velocity [rad/s]', 'meas', 'des');

nexttile;
plot_vector(data, flight, 'desired_wrench.force', {'Fx', 'Fy', 'Fz'}, 1.0);
axis_label('Final body force [N]');

nexttile;
nominal_torque = numeric_fields(data, flight, vector_names('nominal_torque'));
final_torque = numeric_fields(data, flight, ...
    vector_names('desired_wrench.torque'));
plot_pair(time_values(data, flight), nominal_torque, ...
    time_values(data, flight), final_torque, {'Tx', 'Ty', 'Tz'}, ...
    'nominal', 'final');
axis_label('Body torque [N m]', 'nominal', 'final');

nexttile;
plot_vector(data, flight, 'dob_torque', {'Tx', 'Ty', 'Tz'}, 1.0);
axis_label('DOB estimated torque [N m]');

nexttile;
plot_array_pair(data, state, 'rotor_thrust', flight, ...
    'allocated_thrust', 4, 1.0);
axis_label('Rotor thrust [N]', 'meas', 'command');

nexttile;
plot_motor_throttle(data, flight);
axis_label('Motor throttle [%]');
ylim([0, 100]);

nexttile;
plot_array_pair(data, state, 'rotor_tilt', flight, ...
    'allocated_servo_angle', 4, 180.0 / pi);
axis_label('Servo angle [deg]', 'meas', 'command');

nexttile;
time = time_values(data, flight);
residual = numeric_column(data, flight, 'allocation_residual_norm');
saturated = numeric_column(data, flight, 'saturated');
yyaxis left;
plot(time, residual, 'LineWidth', 1.1, 'DisplayName', 'residual');
ylabel('Force residual [N]');
yyaxis right;
stairs(time, saturated, 'LineWidth', 1.0, 'DisplayName', 'saturated');
ylabel('Saturated'); ylim([-0.05, 1.05]);
grid on; xlabel('Time [s]'); title('Allocator health');
end


function plot_arm(data, csv_file)
state = topic_mask(data, '/chr/state');
reference = topic_mask(data, '/chr/reference');

figure('Name', 'CHR - Arm control', 'Color', 'w', ...
    'Position', [80, 80, 1500, 850]);
layout = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, "Arm control | " + string(csv_file), 'Interpreter', 'none');

nexttile;
plot_array_pair(data, state, 'joint_position', reference, ...
    'joint_position', 3, 1.0);
axis_label('Joint position [rad]', 'meas', 'des');

nexttile;
plot_array_tracking_error(data, state, 'joint_position', reference, ...
    'joint_position', 3);
axis_label('Joint position error [rad]');

nexttile;
plot_array_pair(data, state, 'joint_velocity', reference, ...
    'joint_velocity', 3, 1.0);
axis_label('Joint velocity [rad/s]', 'meas', 'des');

for joint = 0:2
    nexttile;
    names = {sprintf('joint_torque_meas[%d]', joint), ...
        sprintf('joint_torque_dyn[%d]', joint), ...
        sprintf('joint_torque_grav[%d]', joint), ...
        sprintf('joint_torque_command[%d]', joint)};
    values = numeric_fields(data, state, names);
    plot(time_values(data, state), values, 'LineWidth', 1.0);
    axis_label(sprintf('J%d torque [N m]', joint + 1));
    legend('meas', 'dyn (M+C+G)', 'grav', 'command', 'Location', 'best');
end
end


function plot_chr(data, csv_file)
state = topic_mask(data, '/chr/state');
reference = topic_mask(data, '/chr/reference');
target = topic_mask(data, '/chr/target/tcp_pose');
ik = topic_mask(data, '/chr/diagnostics/ik');

figure('Name', 'CHR - Whole-body and TCP', 'Color', 'w', ...
    'Position', [120, 80, 1650, 950]);
layout = tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, "CHR whole-body control | " + string(csv_file), ...
    'Interpreter', 'none');

nexttile;
plot_vector_pair(data, state, 'tcp_pose.position', target, ...
    'pose.position', {'x', 'y', 'z'}, 1.0);
axis_label('TCP position [m]', 'meas', 'des');

nexttile;
plot_attitude_pair(data, state, 'tcp_pose.orientation', target, ...
    'pose.orientation');
axis_label('TCP attitude [deg]', 'meas', 'des');

nexttile;
plot_tcp_error_norm(data, state, target);
grid on; xlabel('Time [s]'); title('TCP tracking error');

nexttile;
plot_vector_pair(data, state, 'base_pose.position', reference, ...
    'base_pose.position', {'x', 'y', 'z'}, 1.0);
axis_label('Whole-body base XYZ [m]', 'meas', 'solution');

nexttile;
plot_array_pair(data, state, 'joint_position', reference, ...
    'joint_position', 3, 1.0);
axis_label('Whole-body J1-J3 [rad]', 'meas', 'solution');

nexttile;
time = time_values(data, ik);
position = 1e3 * numeric_column(data, ik, 'position_residual_m');
attitude = rad2deg(numeric_column(data, ik, 'orientation_residual_rad'));
plot(time, position, 'LineWidth', 1.1); hold on;
plot(time, attitude, 'LineWidth', 1.1);
axis_label('DLS residual');
legend('position [mm]', 'attitude [deg]', 'Location', 'best');

nexttile;
time = time_values(data, ik);
minimum = numeric_column(data, ik, 'minimum_singular_value');
condition = numeric_column(data, ik, 'condition_number');
yyaxis left; plot(time, minimum, 'LineWidth', 1.1);
ylabel('Minimum singular value');
yyaxis right; plot(time, condition, 'LineWidth', 1.1);
ylabel('Condition number');
grid on; xlabel('Time [s]'); title('DLS conditioning');

nexttile;
time = time_values(data, ik);
iterations = numeric_column(data, ik, 'iterations');
converged = numeric_column(data, ik, 'converged');
yyaxis left; stairs(time, iterations, 'LineWidth', 1.1);
ylabel('Iterations');
yyaxis right; stairs(time, converged, 'LineWidth', 1.1);
ylabel('Converged'); ylim([-0.05, 1.05]);
grid on; xlabel('Time [s]'); title('DLS convergence');

nexttile;
base_velocity = numeric_fields(data, state, ...
    {'base_twist.linear.x', 'base_twist.linear.y', 'base_twist.linear.z'});
joint_velocity = array_fields(data, state, 'joint_velocity', 3);
time = time_values(data, state);
plot(time, vecnorm(base_velocity, 2, 2), 'LineWidth', 1.1); hold on;
plot(time, vecnorm(joint_velocity, 2, 2), 'LineWidth', 1.1);
axis_label('Base-arm motion coupling');
legend('|v base| [m/s]', '|dq arm| [rad/s]', 'Location', 'best');
end


function plot_vector(data, mask, prefix, labels, scale)
names = vector_names(prefix);
values = scale * numeric_fields(data, mask, names);
plot(time_values(data, mask), values, 'LineWidth', 1.0);
legend(labels, 'Location', 'best');
end


function plot_vector_pair(data, first_mask, first_prefix, second_mask, ...
        second_prefix, labels, scale)
first = scale * numeric_fields(data, first_mask, vector_names(first_prefix));
second = scale * numeric_fields(data, second_mask, vector_names(second_prefix));
plot_pair(time_values(data, first_mask), first, ...
    time_values(data, second_mask), second, labels);
end


function plot_motor_throttle(data, mask)
% Convert the logged PX4-compatible PWM range to normalized throttle.
pwm_min_us = 1100.0;
pwm_max_us = 1900.0;
pwm_us = array_fields(data, mask, 'allocated_pwm_us', 4);
throttle_percent = 100.0 * (pwm_us - pwm_min_us) / ...
    (pwm_max_us - pwm_min_us);
throttle_percent = max(0.0, min(100.0, throttle_percent));
plot(time_values(data, mask), throttle_percent, 'LineWidth', 1.0);
legend(compose('Motor %d', 1:4), 'Location', 'best');
end


function plot_array_pair(data, first_mask, first_prefix, second_mask, ...
        second_prefix, count, scale)
first = scale * array_fields(data, first_mask, first_prefix, count);
second = scale * array_fields(data, second_mask, second_prefix, count);
labels = cellstr(compose('%d', 1:count));
plot_pair(time_values(data, first_mask), first, ...
    time_values(data, second_mask), second, labels);
end


function plot_pair(first_time, first, second_time, second, labels, ...
        first_suffix, second_suffix)
if nargin < 6
    first_suffix = 'meas';
    second_suffix = 'des';
end
colors = lines(size(first, 2));
hold on;
legend_names = strings(1, 2 * size(first, 2));
for index = 1:size(first, 2)
    plot(first_time, first(:, index), '-', 'Color', colors(index, :), ...
        'LineWidth', 1.0);
    plot(second_time, second(:, index), '--', 'Color', colors(index, :), ...
        'LineWidth', 1.0);
    legend_names(2 * index - 1) = ...
        string(labels{index}) + " " + string(first_suffix);
    legend_names(2 * index) = ...
        string(labels{index}) + " " + string(second_suffix);
end
legend(legend_names, 'Location', 'best');
end


function plot_tracking_error(data, measured_mask, measured_prefix, desired_mask, ...
        desired_prefix, labels, scale, wrap_angles)
measured_time = time_values(data, measured_mask);
measured = numeric_fields(data, measured_mask, vector_names(measured_prefix));
desired = numeric_fields(data, desired_mask, vector_names(desired_prefix));
desired_at_measurement = hold_interpolate( ...
    time_values(data, desired_mask), desired, measured_time);
error_value = desired_at_measurement - measured;
if wrap_angles
    error_value = mod(error_value + pi, 2 * pi) - pi;
end
plot(measured_time, scale * error_value, 'LineWidth', 1.0);
legend(labels, 'Location', 'best');
end


function plot_array_tracking_error(data, measured_mask, measured_prefix, ...
        desired_mask, desired_prefix, count)
measured_time = time_values(data, measured_mask);
measured = array_fields(data, measured_mask, measured_prefix, count);
desired = array_fields(data, desired_mask, desired_prefix, count);
desired_at_measurement = hold_interpolate( ...
    time_values(data, desired_mask), desired, measured_time);
plot(measured_time, desired_at_measurement - measured, 'LineWidth', 1.0);
legend(compose('J%d', 1:count), 'Location', 'best');
end


function plot_attitude_pair(data, first_mask, first_prefix, second_mask, second_prefix)
first = quaternion_rpy(data, first_mask, first_prefix);
second = quaternion_rpy(data, second_mask, second_prefix);
plot_pair(time_values(data, first_mask), rad2deg(first), ...
    time_values(data, second_mask), rad2deg(second), {'roll', 'pitch', 'yaw'});
end


function plot_attitude_error(data, measured_mask, measured_prefix, desired_mask, ...
        desired_prefix)
measured_time = time_values(data, measured_mask);
measured = quaternion_rpy(data, measured_mask, measured_prefix);
desired = quaternion_rpy(data, desired_mask, desired_prefix);
desired_at_measurement = hold_interpolate( ...
    time_values(data, desired_mask), desired, measured_time);
error_value = mod(desired_at_measurement - measured + pi, 2 * pi) - pi;
plot(measured_time, rad2deg(error_value), 'LineWidth', 1.0);
legend('roll', 'pitch', 'yaw', 'Location', 'best');
end


function plot_tcp_error_norm(data, state_mask, target_mask)
state_time = time_values(data, state_mask);
position = numeric_fields(data, state_mask, vector_names('tcp_pose.position'));
target_position = numeric_fields(data, target_mask, vector_names('pose.position'));
desired_position = hold_interpolate( ...
    time_values(data, target_mask), target_position, state_time);
position_error = 1e3 * vecnorm(desired_position - position, 2, 2);

attitude = quaternion_values(data, state_mask, 'tcp_pose.orientation');
target_attitude = quaternion_values(data, target_mask, 'pose.orientation');
desired_attitude = hold_interpolate( ...
    time_values(data, target_mask), target_attitude, state_time);
attitude = attitude ./ vecnorm(attitude, 2, 2);
desired_attitude = desired_attitude ./ vecnorm(desired_attitude, 2, 2);
quaternion_dot = abs(sum(attitude .* desired_attitude, 2));
attitude_error = rad2deg(2 * acos(max(-1, min(1, quaternion_dot))));

yyaxis left; plot(state_time, position_error, 'LineWidth', 1.1);
ylabel('Position error [mm]');
yyaxis right; plot(state_time, attitude_error, 'LineWidth', 1.1);
ylabel('Attitude error [deg]');
end


function values = hold_interpolate(source_time, source_values, query_time)
values = nan(numel(query_time), size(source_values, 2));
for column = 1:size(source_values, 2)
    valid = isfinite(source_time) & isfinite(source_values(:, column));
    time = source_time(valid);
    samples = source_values(valid, column);
    [time, unique_indices] = unique(time, 'stable');
    samples = samples(unique_indices);
    if isscalar(time)
        values(query_time >= time, column) = samples;
    elseif numel(time) > 1
        values(:, column) = interp1( ...
            time, samples, query_time, 'previous', NaN);
    end
end
end


function rpy = quaternion_rpy(data, mask, prefix)
quaternion = quaternion_values(data, mask, prefix);
w = quaternion(:, 1); x = quaternion(:, 2);
y = quaternion(:, 3); z = quaternion(:, 4);
roll = atan2(2 .* (w .* x + y .* z), 1 - 2 .* (x.^2 + y.^2));
pitch = asin(max(-1, min(1, 2 .* (w .* y - z .* x))));
yaw = atan2(2 .* (w .* z + x .* y), 1 - 2 .* (y.^2 + z.^2));
rpy = [roll, pitch, yaw];
end


function quaternion = quaternion_values(data, mask, prefix)
prefix = string(prefix);
quaternion = numeric_fields(data, mask, ...
    {prefix + ".w", prefix + ".x", prefix + ".y", prefix + ".z"});
end


function names = vector_names(prefix)
prefix = string(prefix);
names = {prefix + ".x", prefix + ".y", prefix + ".z"};
end


function values = array_fields(data, mask, prefix, count)
names = cell(1, count);
for index = 0:count - 1
    names{index + 1} = sprintf('%s[%d]', prefix, index);
end
values = numeric_fields(data, mask, names);
end


function mask = topic_mask(data, topic)
mask = string(data.topic) == string(topic);
end


function time = time_values(data, mask)
time = numeric_column(data, mask, 'time_s');
end


function values = numeric_fields(data, mask, names)
values = nan(nnz(mask), numel(names));
for index = 1:numel(names)
    values(:, index) = numeric_column(data, mask, names{index});
end
end


function values = numeric_column(data, mask, name)
name = char(string(name));
if ~ismember(name, data.Properties.VariableNames)
    values = nan(nnz(mask), 1);
    return;
end
raw = data{mask, name};
if isnumeric(raw) || islogical(raw)
    values = double(raw);
else
    values = str2double(string(raw));
end
end


function axis_label(text, varargin)
grid on;
xlabel('Time [s]');
ylabel(text);
if ~isempty(varargin)
    title(strjoin(string(varargin), ' / '));
end
end
