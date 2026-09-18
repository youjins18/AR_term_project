function chr_bag_plot(csv_file)
%CHR_BAG_PLOT Plot flight, arm, and whole-body CHR diagnostics.
% Run without an argument to select a CSV exported by bag_to_csv.py.
close all;
clear all;
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

plot_overview(data, csv_file);
plot_chr(data, csv_file);
plot_palletrone_pose(data, csv_file);
plot_palletrone_velocity(data, csv_file);
plot_palletrone_wrench(data, csv_file);
plot_palletrone_actuators(data, csv_file);
plot_arm_motion(data, csv_file);
plot_arm_torque(data, csv_file);
print_tracking_review(data, csv_file);
end


function plot_overview(data, csv_file)
state = topic_mask(data, '/chr/state');
target = topic_mask(data, '/chr/target/tcp_pose');

dashboard('Figure 1 - Overview dashboard', ...
    'CHR control overview', csv_file, 2, 3);

axis_names = {'X', 'Y', 'Z'};
for axis_index = 1:3
    nexttile;
    plot_vector_component_pair(data, state, 'tcp_pose.position', target, ...
        'pose.position', axis_index, 1.0);
    finish_axis('TCP position [m]', "TCP " + axis_names{axis_index});
end

angle_names = {'TCP roll', 'TCP pitch', 'TCP yaw'};
for axis_index = 1:3
    nexttile;
    plot_attitude_component_pair(data, state, 'tcp_pose.orientation', ...
        target, 'pose.orientation', axis_index);
    finish_axis('TCP attitude [deg]', angle_names{axis_index});
end
link_time_axes();
end


function plot_palletrone_pose(data, csv_file)
state = topic_mask(data, '/chr/state');
reference = topic_mask(data, '/chr/reference');
dashboard('Figure 3 - Palletrone pose', ...
    'Palletrone position and attitude', csv_file, 2, 3);

axes_xyz = {'X', 'Y', 'Z'};
for axis_index = 1:3
    nexttile;
    plot_vector_component_pair(data, state, 'base_pose.position', reference, ...
        'base_pose.position', axis_index, 1.0);
    finish_axis('Position [m]', axes_xyz{axis_index});
end

angle_names = {'Roll', 'Pitch', 'Yaw'};
for axis_index = 1:3
    nexttile;
    plot_attitude_component_pair(data, state, 'base_pose.orientation', ...
        reference, 'base_pose.orientation', axis_index);
    finish_axis('Attitude [deg]', angle_names{axis_index});
end
link_time_axes();
end


function plot_palletrone_velocity(data, csv_file)
state = topic_mask(data, '/chr/state');
reference = topic_mask(data, '/chr/reference');
dashboard('Figure 4 - Palletrone velocity', ...
    'Palletrone linear and angular velocity', csv_file, 2, 3);

axes_xyz = {'Vx', 'Vy', 'Vz'};
for axis_index = 1:3
    nexttile;
    plot_vector_component_pair(data, state, 'base_twist.linear', reference, ...
        'base_twist.linear', axis_index, 1.0);
    finish_axis('Velocity [m/s]', axes_xyz{axis_index});
end

rate_names = {'p', 'q', 'r'};
for axis_index = 1:3
    nexttile;
    plot_vector_component_pair(data, state, 'base_twist.angular', reference, ...
        'base_twist.angular', axis_index, 1.0);
    finish_axis('Angular velocity [rad/s]', rate_names{axis_index});
end
link_time_axes();
end


function plot_palletrone_wrench(data, csv_file)
flight = topic_mask(data, '/chr/diagnostics/flight');
dashboard('Figure 5 - Palletrone wrench', ...
    'Palletrone force, torque and DOB', csv_file, 2, 3);

force_names = {'F_x', 'F_y', 'F_z'};
for axis_index = 1:3
    nexttile;
    plot_vector_component(data, flight, 'desired_wrench.force', axis_index);
    finish_axis('Body force [N]', force_names{axis_index});
end

torque_names = {'T_x', 'T_y', 'T_z'};
for axis_index = 1:3
    nexttile;
    plot_torque_component(data, flight, axis_index);
    finish_axis('Body torque [N m]', torque_names{axis_index});
end
link_time_axes();
end


function plot_palletrone_actuators(data, csv_file)
state = topic_mask(data, '/chr/state');
flight = topic_mask(data, '/chr/diagnostics/flight');
dashboard('Figure 6 - Palletrone actuators', ...
    'Palletrone actuator commands', csv_file, 2, 3);

nexttile;
plot_array(data, flight, 'allocated_thrust', 4, 1.0, 'Rotor');
finish_axis('Thrust command [N]', 'Rotor thrust');

nexttile;
plot_motor_throttle(data, flight);
finish_axis('Throttle [%]', 'Motor throttle');
ylim([0, 100]);

for rotor = 1:4
    nexttile;
    plot_array_component_pair(data, state, 'rotor_tilt', flight, ...
        'allocated_servo_angle', rotor, 180.0 / pi);
    finish_axis('Servo angle [deg]', sprintf('Servo %d', rotor));
end
link_time_axes();
end


function plot_arm_motion(data, csv_file)
state = topic_mask(data, '/chr/state');
reference = topic_mask(data, '/chr/reference');

dashboard('Figure 7 - Arm motion', ...
    'Arm joint position and velocity', csv_file, 2, 3);

for joint = 1:3
    nexttile;
    plot_array_component_pair(data, state, 'joint_position', reference, ...
        'joint_position', joint, 1.0);
    finish_axis('Position [rad]', sprintf('J%d position', joint));
end
for joint = 1:3
    nexttile;
    plot_array_component_pair(data, state, 'joint_velocity', reference, ...
        'joint_velocity', joint, 1.0);
    finish_axis('Velocity [rad/s]', sprintf('J%d velocity', joint));
end
link_time_axes();
end


function plot_arm_torque(data, csv_file)
state = topic_mask(data, '/chr/state');
dashboard('Figure 8 - Arm torque', ...
    'Arm joint torque', csv_file, 1, 3);

for joint = 0:2
    nexttile;
    names = {sprintf('joint_torque_meas[%d]', joint), ...
        sprintf('joint_torque_dyn[%d]', joint), ...
        sprintf('joint_torque_grav[%d]', joint), ...
        sprintf('joint_torque_command[%d]', joint)};
    values = numeric_fields(data, state, names);
    colors = plot_palette();
    plot(time_values(data, state), values(:, 1), ...
        'Color', colors.measured, 'LineWidth', 1.15); hold on;
    plot(time_values(data, state), values(:, 2), ...
        'Color', colors.dynamic, 'LineWidth', 1.05);
    plot(time_values(data, state), values(:, 3), '--', ...
        'Color', colors.gravity, 'LineWidth', 1.05);
    plot(time_values(data, state), values(:, 4), ':', ...
        'Color', colors.command, 'LineWidth', 1.25);
    finish_axis('Torque [N m]', sprintf('J%d torque', joint + 1));
    clean_legend({'meas', 'dyn (M+C+G)', 'gravity', 'command'});
end
link_time_axes();
end


function plot_chr(data, csv_file)
state = topic_mask(data, '/chr/state');
ik = topic_mask(data, '/chr/diagnostics/ik');

dashboard('Figure 2 - CHR', ...
    'CHR DLS-IK and base-arm motion', csv_file, 1, 3);

nexttile;
time = time_values(data, ik);
minimum = numeric_column(data, ik, 'minimum_singular_value');
condition = numeric_column(data, ik, 'condition_number');
colors = plot_palette();
yyaxis left;
plot(time, minimum, 'Color', colors.measured, 'LineWidth', 1.15);
ylabel('Minimum singular value');
yyaxis right;
plot(time, condition, 'Color', colors.command, 'LineWidth', 1.15);
ylabel('Condition number');
finish_axis('', 'DLS conditioning');

nexttile;
time = time_values(data, ik);
iterations = numeric_column(data, ik, 'iterations');
converged = numeric_column(data, ik, 'converged');
yyaxis left;
stairs(time, iterations, 'Color', colors.measured, 'LineWidth', 1.15);
ylabel('Iterations');
yyaxis right;
stairs(time, converged, 'Color', colors.command, 'LineWidth', 1.15);
ylabel('Converged'); ylim([-0.05, 1.05]);
finish_axis('', 'DLS convergence');

nexttile;
base_velocity = numeric_fields(data, state, ...
    {'base_twist.linear.x', 'base_twist.linear.y', 'base_twist.linear.z'});
joint_velocity = array_fields(data, state, 'joint_velocity', 3);
time = time_values(data, state);
plot(time, vecnorm(base_velocity, 2, 2), ...
    'Color', colors.measured, 'LineWidth', 1.15); hold on;
plot(time, vecnorm(joint_velocity, 2, 2), ...
    'Color', colors.desired, 'LineWidth', 1.15);
finish_axis('Norm', 'Base-arm motion');
clean_legend({'|v_{base}| [m/s]', '|dq_{arm}| [rad/s]'});
link_time_axes();
end


function dashboard(window_name, heading, csv_file, rows, columns)
[~, source_name, extension] = fileparts(string(csv_file));
figure_width = 0.72;
figure_height = 0.34 * rows;
figure_left = (1.0 - figure_width) / 2.0;
figure_bottom = (1.0 - figure_height) / 2.0;
figure('Name', window_name, 'NumberTitle', 'off', 'Color', 'w', ...
    'Units', 'normalized', ...
    'Position', [figure_left, figure_bottom, figure_width, figure_height]);
layout = tiledlayout(rows, columns, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, string(heading) + " | " + string(source_name) + string(extension), ...
    'Interpreter', 'none', 'FontWeight', 'bold');
end


function plot_vector_component_pair(data, measured_mask, measured_prefix, ...
        desired_mask, desired_prefix, component, scale)
measured_names = vector_names(measured_prefix);
desired_names = vector_names(desired_prefix);
measured = scale * numeric_column(data, measured_mask, ...
    measured_names{component});
desired = scale * numeric_column(data, desired_mask, ...
    desired_names{component});
plot_signal_pair(time_values(data, measured_mask), measured, ...
    time_values(data, desired_mask), desired);
end


function plot_array_component_pair(data, measured_mask, measured_prefix, ...
        desired_mask, desired_prefix, component, scale)
measured = scale * array_fields(data, measured_mask, measured_prefix, component);
desired = scale * array_fields(data, desired_mask, desired_prefix, component);
plot_signal_pair(time_values(data, measured_mask), measured(:, component), ...
    time_values(data, desired_mask), desired(:, component));
end


function plot_attitude_component_pair(data, measured_mask, measured_prefix, ...
        desired_mask, desired_prefix, component)
measured = unwrap(quaternion_rpy(data, measured_mask, measured_prefix), [], 1);
desired = unwrap(quaternion_rpy(data, desired_mask, desired_prefix), [], 1);
measured_angle = align_angle_branch(measured(:, component), desired(:, component));
desired_angle = desired(:, component);
plot_signal_pair(time_values(data, measured_mask), rad2deg(measured_angle), ...
    time_values(data, desired_mask), rad2deg(desired_angle));
end


function aligned = align_angle_branch(values, reference)
aligned = values;
value_index = find(isfinite(values), 1);
reference_index = find(isfinite(reference), 1);
if isempty(value_index) || isempty(reference_index)
    return;
end
aligned = values + 2.0 * pi * round( ...
    (reference(reference_index) - values(value_index)) / (2.0 * pi));
end


function plot_signal_pair(measured_time, measured, desired_time, desired)
colors = plot_palette();
plot(measured_time, measured, 'Color', colors.measured, ...
    'LineWidth', 1.2); hold on;
plot(desired_time, desired, '--', 'Color', colors.desired, ...
    'LineWidth', 1.2);
clean_legend({'meas', 'des'});
end


function plot_vector_component(data, mask, prefix, component)
names = vector_names(prefix);
colors = plot_palette();
plot(time_values(data, mask), numeric_column(data, mask, names{component}), ...
    'Color', colors.command, 'LineWidth', 1.2);
end


function plot_torque_component(data, mask, component)
names = vector_names('nominal_torque');
nominal = numeric_column(data, mask, names{component});
names = vector_names('desired_wrench.torque');
final_value = numeric_column(data, mask, names{component});
names = vector_names('dob_torque');
dob = numeric_column(data, mask, names{component});
colors = plot_palette();
time = time_values(data, mask);
plot(time, nominal, '--', 'Color', colors.nominal, ...
    'LineWidth', 1.05); hold on;
plot(time, final_value, 'Color', colors.command, 'LineWidth', 1.2);
plot(time, dob, ':', 'Color', colors.dob, 'LineWidth', 1.25);
clean_legend({'nominal', 'final', 'DOB'});
end


function plot_array(data, mask, prefix, count, scale, label_prefix)
values = scale * array_fields(data, mask, prefix, count);
plot(time_values(data, mask), values, 'LineWidth', 1.1);
clean_legend(cellstr(string(label_prefix) + " " + string(1:count)));
end


function plot_motor_throttle(data, mask)
pwm_min_us = 1100.0;
pwm_max_us = 1900.0;
pwm_us = array_fields(data, mask, 'allocated_pwm_us', 4);
throttle_percent = 100.0 * (pwm_us - pwm_min_us) / ...
    (pwm_max_us - pwm_min_us);
throttle_percent = max(0.0, min(100.0, throttle_percent));
plot(time_values(data, mask), throttle_percent, 'LineWidth', 1.1);
clean_legend(cellstr(compose('Motor %d', 1:4)));
end


function print_tracking_review(data, csv_file)
state = topic_mask(data, '/chr/state');
reference = topic_mask(data, '/chr/reference');
target = topic_mask(data, '/chr/target/tcp_pose');
flight = topic_mask(data, '/chr/diagnostics/flight');
ik = topic_mask(data, '/chr/diagnostics/ik');

fprintf('\n============================================================\n');
fprintf('CHR tracking review\n');
fprintf('CSV: %s\n', char(string(csv_file)));
fprintf('------------------------------------------------------------\n');

base_position_error = vector_tracking_error(data, state, ...
    'base_pose.position', reference, 'base_pose.position');
print_norm_stat('Base position', base_position_error, 1e3, 'mm');

base_attitude_error = quaternion_tracking_error(data, state, ...
    'base_pose.orientation', reference, 'base_pose.orientation');
print_scalar_stat('Base attitude', base_attitude_error, 180 / pi, 'deg');

tcp_position_error = vector_tracking_error(data, state, ...
    'tcp_pose.position', target, 'pose.position');
print_norm_stat('TCP position', tcp_position_error, 1e3, 'mm');

tcp_attitude_error = quaternion_tracking_error(data, state, ...
    'tcp_pose.orientation', target, 'pose.orientation');
print_scalar_stat('TCP attitude', tcp_attitude_error, 180 / pi, 'deg');

joint_error = array_tracking_error(data, state, 'joint_position', ...
    reference, 'joint_position', 3);
print_norm_stat('Arm joint position', joint_error, 180 / pi, 'deg');

active = numeric_column(data, ik, 'target_active') > 0.5;
dls_position = numeric_column(data, ik, 'position_residual_m');
dls_attitude = numeric_column(data, ik, 'orientation_residual_rad');
dls_position(~active) = NaN;
dls_attitude(~active) = NaN;
print_scalar_stat('DLS position residual', dls_position, 1e3, 'mm');
print_scalar_stat('DLS attitude residual', dls_attitude, 180 / pi, 'deg');

converged = numeric_column(data, ik, 'converged');
valid = active & isfinite(converged);
if any(valid)
    fprintf('%-27s %8.2f %%\n', 'DLS convergence', ...
        100 * mean(converged(valid) > 0.5));
else
    fprintf('%-27s %s\n', 'DLS convergence', 'n/a');
end

allocation_residual = numeric_column(data, flight, ...
    'allocation_residual_norm');
print_scalar_stat('Allocator residual', allocation_residual, 1.0, 'N');
saturated = numeric_column(data, flight, 'saturated');
valid = isfinite(saturated);
if any(valid)
    fprintf('%-27s %8.2f %%\n', 'Allocator saturation', ...
        100 * mean(saturated(valid) > 0.5));
else
    fprintf('%-27s %s\n', 'Allocator saturation', 'n/a');
end
fprintf('============================================================\n\n');
end


function error_value = vector_tracking_error(data, measured_mask, ...
        measured_prefix, desired_mask, desired_prefix)
measured_time = time_values(data, measured_mask);
measured = numeric_fields(data, measured_mask, vector_names(measured_prefix));
desired = numeric_fields(data, desired_mask, vector_names(desired_prefix));
desired = hold_interpolate(time_values(data, desired_mask), desired, measured_time);
error_value = desired - measured;
end


function error_value = array_tracking_error(data, measured_mask, ...
        measured_prefix, desired_mask, desired_prefix, count)
measured_time = time_values(data, measured_mask);
measured = array_fields(data, measured_mask, measured_prefix, count);
desired = array_fields(data, desired_mask, desired_prefix, count);
desired = hold_interpolate(time_values(data, desired_mask), desired, measured_time);
error_value = desired - measured;
end


function error_angle = quaternion_tracking_error(data, measured_mask, ...
        measured_prefix, desired_mask, desired_prefix)
measured_time = time_values(data, measured_mask);
measured = quaternion_values(data, measured_mask, measured_prefix);
desired = quaternion_values(data, desired_mask, desired_prefix);
desired = hold_interpolate(time_values(data, desired_mask), desired, measured_time);

measured_norm = vecnorm(measured, 2, 2);
desired_norm = vecnorm(desired, 2, 2);
valid = all(isfinite(measured), 2) & all(isfinite(desired), 2) & ...
    measured_norm > eps & desired_norm > eps;
error_angle = nan(size(measured, 1), 1);
measured(valid, :) = measured(valid, :) ./ measured_norm(valid);
desired(valid, :) = desired(valid, :) ./ desired_norm(valid);
quaternion_dot = abs(sum(measured(valid, :) .* desired(valid, :), 2));
error_angle(valid) = 2 * acos(max(-1, min(1, quaternion_dot)));
end


function print_norm_stat(label, error_value, scale, unit)
norm_value = scale * vecnorm(error_value, 2, 2);
print_scalar_stat(label, norm_value, 1.0, unit);
end


function print_scalar_stat(label, values, scale, unit)
values = scale * values(isfinite(values));
if isempty(values)
    fprintf('%-27s %s\n', label, 'n/a');
    return;
end
rms_value = sqrt(mean(values.^2));
fprintf('%-27s RMS %9.3f %-4s | max %9.3f %s\n', ...
    label, rms_value, unit, max(abs(values)), unit);
end


function finish_axis(y_label, title_text)
ax = gca;
grid(ax, 'on');
box(ax, 'on');
ax.FontName = 'Helvetica';
ax.FontSize = 10;
ax.LineWidth = 0.8;
ax.GridAlpha = 0.16;
xlabel(ax, 'Time [s]');
if strlength(string(y_label)) > 0
    ylabel(ax, y_label);
end
title(ax, title_text, 'FontWeight', 'normal');
end


function clean_legend(labels)
legend(labels, 'Location', 'best', 'Box', 'off', 'FontSize', 8);
end


function colors = plot_palette()
colors.measured = [0.10, 0.33, 0.65];
colors.desired = [0.90, 0.32, 0.12];
colors.nominal = [0.35, 0.35, 0.35];
colors.command = [0.10, 0.55, 0.35];
colors.dob = [0.55, 0.20, 0.65];
colors.dynamic = [0.92, 0.48, 0.08];
colors.gravity = [0.48, 0.25, 0.65];
end


function link_time_axes()
axes_list = findall(gcf, 'Type', 'axes');
if numel(axes_list) > 1
    linkaxes(axes_list, 'x');
end
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
