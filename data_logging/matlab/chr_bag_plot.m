function chr_bag_plot(csv_file)
%CHR_BAG_PLOT Plot the primary CHR states and references from an exported bag.
% Run without an argument to choose a CSV file from a dialog.

if nargin < 1 || strlength(string(csv_file)) == 0
    [file, folder] = uigetfile('*.csv', 'Select a CHR bag CSV');
    if isequal(file, 0)
        return;
    end
    csv_file = fullfile(folder, file);
end

data = readtable(csv_file, 'VariableNamingRule', 'preserve');
topics = string(data.topic);
state = topics == "/chr/state";
reference = topics == "/chr/reference";
target = topics == "/chr/target/tcp_pose";

figure('Name', 'CHR bag overview', 'Color', 'w');
layout = tiledlayout(2, 2, 'TileSpacing', 'compact');
title(layout, string(csv_file), 'Interpreter', 'none');

nexttile;
plot_xyz(data, state, 'base_pose.position', '-');
hold on;
plot_xyz(data, reference, 'base_pose.position', '--');
grid on; ylabel('base position [m]');
legend('x', 'y', 'z', 'x ref', 'y ref', 'z ref', 'Location', 'best');

nexttile;
plot_rpy(data, state, 'base_pose.orientation', '-');
hold on;
plot_rpy(data, reference, 'base_pose.orientation', '--');
grid on; ylabel('base attitude [deg]');
legend('roll', 'pitch', 'yaw', 'roll ref', 'pitch ref', 'yaw ref', ...
    'Location', 'best');

nexttile;
plot_fields(data, state, ...
    {'joint_position[0]', 'joint_position[1]', 'joint_position[2]'}, '-');
hold on;
plot_fields(data, reference, ...
    {'joint_position[0]', 'joint_position[1]', 'joint_position[2]'}, '--');
grid on; ylabel('joint position [rad]'); xlabel('time [s]');
legend('J1', 'J2', 'J3', 'J1 ref', 'J2 ref', 'J3 ref', 'Location', 'best');

nexttile;
plot_xyz(data, state, 'tcp_pose.position', '-');
hold on;
plot_xyz(data, target, 'pose.position', '--');
grid on; ylabel('TCP position [m]'); xlabel('time [s]');
legend('x', 'y', 'z', 'x target', 'y target', 'z target', 'Location', 'best');
end


function plot_xyz(data, mask, prefix, style)
prefix = string(prefix);
plot_fields(data, mask, ...
    {prefix + ".x", prefix + ".y", prefix + ".z"}, style);
end


function plot_rpy(data, mask, prefix, style)
prefix = string(prefix);
quaternion = numeric_fields(data, mask, ...
    {prefix + ".w", prefix + ".x", prefix + ".y", prefix + ".z"});
w = quaternion(:, 1); x = quaternion(:, 2);
y = quaternion(:, 3); z = quaternion(:, 4);
roll = atan2(2 .* (w .* x + y .* z), 1 - 2 .* (x.^2 + y.^2));
pitch = asin(max(-1, min(1, 2 .* (w .* y - z .* x))));
yaw = atan2(2 .* (w .* z + x .* y), 1 - 2 .* (y.^2 + z.^2));
plot(time_values(data, mask), rad2deg([roll, pitch, yaw]), style);
end


function plot_fields(data, mask, names, style)
plot(time_values(data, mask), numeric_fields(data, mask, names), style);
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
if ~ismember(name, data.Properties.VariableNames)
    values = nan(nnz(mask), 1);
    return;
end
raw = data{mask, name};
if isnumeric(raw)
    values = double(raw);
else
    values = str2double(string(raw));
end
end
