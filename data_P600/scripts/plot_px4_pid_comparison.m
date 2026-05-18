function report = plot_px4_pid_comparison(mode1_eso_file, mode2_eso_file, output_dir)
%PLOT_PX4_PID_COMPARISON Plot P600 real-flight ESO results only.

if nargin < 3 || isempty(output_dir)
    output_dir = fullfile(pwd, 'figures', 'p600_real');
end
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

cases = {
    'mode1', 'paper_eso', '本文算法', mode1_eso_file, '扰动抑制实验';
    'mode2', 'paper_eso', '本文算法', mode2_eso_file, '轨迹跟踪实验'};

report = struct();
for i = 1:size(cases, 1)
    loaded = load(cases{i, 4}, 'result');
    data = extract_case_data(loaded.result);
    report.(cases{i, 1}).(cases{i, 2}) = loaded.result.metrics;
    report.(cases{i, 1}).([cases{i, 2} '_data']) = data;
    report.(cases{i, 1}).([cases{i, 2} '_label']) = cases{i, 3};
    plot_mode_single(data, output_dir, cases{i, 1}, cases{i, 5});
end
write_metric_table(report, output_dir);

function data = extract_case_data(result)
signals = result.signals;
data.p_true = signal_data(signals, 'p_true');
data.p_actual = fallback_signal(signals, 'p_actual', data.p_true);
data.p_desired = signal_data(signals, 'p_desired');
data.q_true = signal_data(signals, 'q_true');
data.q_desired = signal_data(signals, 'q_desired');
data.f = signal_data(signals, 'f');
data.tau = signal_data(signals, 'tau');
data.metrics = result.metrics;
data.t = signal_time(signals, 'p_true', size(data.p_true, 1));
trim_tail_seconds = 2;

n = min([numel(data.t), size(data.p_true, 1), size(data.p_desired, 1), ...
    size(data.q_true, 1), size(data.q_desired, 1)]);
fields = fieldnames(data);
for i = 1:numel(fields)
    value = data.(fields{i});
    if isnumeric(value) && size(value, 1) >= n
        data.(fields{i}) = value(1:n, :);
    end
end
if n > 1
    keep = data.t <= data.t(end) - trim_tail_seconds;
    if any(keep)
        fields = fieldnames(data);
        for i = 1:numel(fields)
            value = data.(fields{i});
            if isnumeric(value) && size(value, 1) == n
                data.(fields{i}) = value(keep, :);
            end
        end
    end
end
data.p_error = data.p_true - data.p_desired;
data.q_error = data.q_true - data.q_desired;
data.position_error_norm = vecnorm(data.p_error, 2, 2);

function plot_mode_single(eso, output_dir, mode_label, experiment_title)
position_labels = {'p_x / m', 'p_y / m', 'p_z / m'};
position_error_labels = {'e_x / mm', 'e_y / mm', 'e_z / mm'};
joint_labels = {'q_1 / rad', 'q_2 / rad', 'q_3 / rad'};
time_label = 't / s';

style = struct();
style.main_line_width = 1.76;
style.reference_line_width = 1.43;
style.mean_line_width = 1.43;
style.english_font_name = 'Times New Roman';
style.chinese_font_name = 'SimHei';
style.legend_font_size = 16.5;
style.label_font_size = 17.6;
style.axis_font_size = 15.4;
style.axis_line_width = 1.0;
style.legend_font_weight = 'bold';
style.label_font_weight = 'bold';
style.axis_font_weight = 'bold';
style.x_ticks = 0:10:max(ceil(max(eso.t) / 10) * 10, 10);
style.y_padding_ratio = 0.08;
style.min_span_position_m = 0.002;
style.min_span_error_mm = 0.8;
style.min_span_joint_rad = 0.02;

position_error_mm_eso = eso.p_error * 1e3;
position_error_norm_mm_eso = eso.position_error_norm * 1e3;

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1720 1220]);
layout = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
track_handles = gobjects(1, 2);
for i = 1:3
    ax_track = nexttile(layout, (i - 1) * 2 + 1);
    track_handles(1) = plot(ax_track, eso.t, eso.p_true(:, i), 'b-', ...
        'LineWidth', style.main_line_width); hold(ax_track, 'on');
    track_handles(2) = plot(ax_track, eso.t, eso.p_desired(:, i), 'k--', ...
        'LineWidth', style.reference_line_width);
    configure_2d_axes(ax_track, position_labels{i}, i == 3, time_label, style);
    apply_data_ylim(ax_track, [eso.p_true(:, i); eso.p_desired(:, i)], ...
        style.y_padding_ratio, style.min_span_position_m);

    ax_error = nexttile(layout, (i - 1) * 2 + 2);
    plot(ax_error, eso.t, position_error_mm_eso(:, i), 'b-', ...
        'LineWidth', style.main_line_width);
    configure_2d_axes(ax_error, position_error_labels{i}, i == 3, time_label, style);
    apply_data_ylim(ax_error, position_error_mm_eso(:, i), ...
        style.y_padding_ratio, style.min_span_error_mm);
end
lgd = legend(track_handles, {legend_text('本文算法', false, style), ...
    legend_text('期望值', false, style)}, ...
    'Orientation', 'horizontal', 'FontSize', style.legend_font_size, ...
    'FontWeight', style.legend_font_weight, 'FontName', style.english_font_name, ...
    'Interpreter', 'tex', 'Box', 'off');
lgd.Layout.Tile = 'north';
save_figure(fig, output_dir, sprintf('%s_position_tracking_and_error_p600_real.png', mode_label));

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1720 720]);
layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'loose', 'Padding', 'loose');
ax_traj = nexttile(layout, 1);
h_traj(1) = plot3(ax_traj, eso.p_true(:, 1), eso.p_true(:, 2), eso.p_true(:, 3), 'b-', ...
    'LineWidth', style.main_line_width); hold(ax_traj, 'on');
h_traj(2) = plot3(ax_traj, eso.p_desired(:, 1), eso.p_desired(:, 2), eso.p_desired(:, 3), 'k--', ...
    'LineWidth', style.reference_line_width);
grid(ax_traj, 'on');
axis(ax_traj, 'equal');
set(ax_traj, 'FontSize', style.axis_font_size, 'LineWidth', style.axis_line_width, ...
    'FontWeight', style.axis_font_weight, 'FontName', style.english_font_name, ...
    'Box', 'on', 'Layer', 'top');
xlabel(ax_traj, 'p_x / m', 'FontSize', style.label_font_size, ...
    'FontWeight', style.label_font_weight, 'FontName', style.english_font_name);
ylabel(ax_traj, 'p_y / m', 'FontSize', style.label_font_size, ...
    'FontWeight', style.label_font_weight, 'FontName', style.english_font_name);
zlabel(ax_traj, 'p_z / m', 'FontSize', style.label_font_size, ...
    'FontWeight', style.label_font_weight, 'FontName', style.english_font_name);
z_limits = [min([eso.p_true(:, 3); eso.p_desired(:, 3)]), ...
    max([eso.p_true(:, 3); eso.p_desired(:, 3)])];
zlim(ax_traj, z_limits + [-1 1] * max(0.001, 0.12 * diff(z_limits)));
zticks(ax_traj, [z_limits(1), z_limits(2)]);
ztickformat(ax_traj, '%.2f');
ax_traj.ZAxis.FontSize = max(style.axis_font_size - 2, 10);
view(ax_traj, [-138 22]);
pbaspect(ax_traj, [1.35 1.35 0.32]);

ax_norm = nexttile(layout, 2);
h_norm(1) = plot(ax_norm, eso.t, position_error_norm_mm_eso, 'b-', ...
    'LineWidth', style.main_line_width); hold(ax_norm, 'on');
h_norm(2) = yline(ax_norm, mean(position_error_norm_mm_eso), 'k--', ...
    'LineWidth', style.mean_line_width);
configure_2d_axes(ax_norm, 'e_p / mm', true, time_label, style);
apply_data_ylim(ax_norm, position_error_norm_mm_eso, ...
    style.y_padding_ratio, style.min_span_error_mm);
lgd = legend(h_norm, {legend_text('本文算法', false, style), ...
    legend_text('Mean', true, style)}, ...
    'Orientation', 'horizontal', 'FontSize', style.legend_font_size, ...
    'FontWeight', style.legend_font_weight, 'FontName', style.english_font_name, ...
    'Interpreter', 'tex', 'Box', 'off');
lgd.Layout.Tile = 'north';
save_figure(fig, output_dir, sprintf('%s_3d_mean_error_p600_real.png', mode_label));

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1180 1160]);
layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
joint_handles = gobjects(1, 1);
for i = 1:3
    ax_joint = nexttile(layout, i);
    joint_handles(1) = plot(ax_joint, eso.t, eso.q_true(:, i), 'b-', ...
        'LineWidth', style.main_line_width);
    configure_2d_axes(ax_joint, joint_labels{i}, i == 3, time_label, style);
    apply_data_ylim(ax_joint, eso.q_true(:, i), ...
        style.y_padding_ratio, style.min_span_joint_rad);
end
lgd = legend(joint_handles, {legend_text('本文算法', false, style)}, ...
    'Orientation', 'horizontal', 'FontSize', style.legend_font_size, ...
    'FontWeight', style.legend_font_weight, 'FontName', style.english_font_name, ...
    'Interpreter', 'tex', 'Box', 'off');
lgd.String = {legend_text('机械臂跟踪', false, style)};
lgd.Layout.Tile = 'north';
lgd.String = {legend_text(char([26426 26800 33218 36319 36394]), false, style)};
save_figure(fig, output_dir, sprintf('%s_arm_tracking_p600_real.png', mode_label));

function configure_2d_axes(ax, y_label_text, show_x_label, x_label_text, style)
grid(ax, 'on');
set(ax, 'FontSize', style.axis_font_size, 'LineWidth', style.axis_line_width, ...
    'FontWeight', style.axis_font_weight, 'FontName', style.english_font_name, ...
    'Box', 'on', 'Layer', 'top', 'XTick', style.x_ticks);
ylabel(ax, y_label_text, 'FontSize', style.label_font_size, ...
    'FontWeight', style.label_font_weight, 'FontName', style.english_font_name);
if show_x_label
    xlabel(ax, x_label_text, 'FontSize', style.label_font_size, ...
        'FontWeight', style.label_font_weight, 'FontName', style.english_font_name);
else
    ax.XTickLabel = [];
end

function text = legend_text(content, is_english, style)
if is_english
    font_name = style.english_font_name;
else
    font_name = style.chinese_font_name;
end
text = ['\fontname{' font_name '}\bf ' content];

function apply_data_ylim(ax, values, padding_ratio, min_span)
values = values(isfinite(values));
if isempty(values)
    ylim(ax, [-1 1]);
    return;
end
v_min = min(values);
v_max = max(values);
span = max(v_max - v_min, min_span);
center = 0.5 * (v_min + v_max);
half_span = 0.5 * span * (1 + 2 * padding_ratio);
if half_span <= 0
    half_span = 0.5 * min_span;
end
ylim(ax, [center - half_span, center + half_span]);

function write_metric_table(report, output_dir)
fid = fopen(fullfile(output_dir, 'p600_real_metrics_summary.txt'), 'w');
if fid < 0
    error('Unable to write P600 metric summary');
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '控制器,实验,平均位置误差,各轴平均位置误差,各轴最大位置误差,位置均方根误差,最大位置误差,机械臂各轴最大误差,是否发散,推力饱和比例,力矩饱和比例\n');
write_metrics(fid, '本文算法', '扰动抑制实验', report.mode1.paper_eso);
write_metrics(fid, '本文算法', '轨迹跟踪实验', report.mode2.paper_eso);

function write_metrics(fid, controller, mode_label, metrics)
fprintf(fid, '%s,%s,%.6f,%s,%s,%.6f,%.6f,%s,%d,%.6f,%.6f\n', ...
    controller, mode_label, scalar_field(metrics, 'position_mean'), ...
    vec_to_string(metrics.position_axis_mean), vec_to_string(metrics.position_axis_max), ...
    metrics.position_rms, metrics.position_max, vec_to_string(metrics.arm_axis_max), ...
    metrics.is_divergent, metrics.thrust_saturation_ratio, ...
    metrics.torque_saturation_ratio);

function value = scalar_field(s, name)
if isfield(s, name)
    value = s.(name);
else
    value = NaN;
end

function text = vec_to_string(v)
text = sprintf('[%.6f %.6f %.6f]', v(1), v(2), v(3));

function data = fallback_signal(signals, name, fallback)
data = signal_data(signals, name);
if isempty(data)
    data = fallback;
end

function data = signal_data(signals, name)
data = [];
if isfield(signals, name)
    value = signals.(name);
    if isstruct(value) && isfield(value, 'data')
        data = value.data;
    elseif isnumeric(value)
        data = value;
    end
end

function t = signal_time(signals, name, sample_count)
if isfield(signals, name) && isfield(signals.(name), 'time')
    t = signals.(name).time(:);
else
    t = (0:sample_count - 1)' * 0.01;
end

function save_figure(fig, output_dir, filename)
[~, name, ~] = fileparts(filename);
exportgraphics(fig, fullfile(output_dir, [name '.png']), 'Resolution', 900);
close(fig);
