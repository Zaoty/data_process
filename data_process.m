% -------------------------------------------------------------------------
% 1. Data Extraction & Alignment (修正版)
% -------------------------------------------------------------------------
disp('Loading Rosbags and Extracting Data...');

% --- Dataset 1: Proposed Method (With Pre-config) ---
bag1 = rosbag('F:\Home\practice_data\franka_catch_full_opti_record_consider.bag');

% Force
f_msgs1 = select(bag1, 'Topic', '/franka_state_controller/F_ext');
f_structs1 = readMessages(f_msgs1, 'DataFormat', 'struct');
t1 = f_msgs1.MessageList.Time - f_msgs1.MessageList.Time(1);

% 【修正】分别提取分量，再计算模长 (更安全、更快)
Fx1 = cellfun(@(m) m.Wrench.Force.X, f_structs1);
Fy1 = cellfun(@(m) m.Wrench.Force.Y, f_structs1);
Fz1 = cellfun(@(m) m.Wrench.Force.Z, f_structs1);
F_norm1 = sqrt(Fx1.^2 + Fy1.^2 + Fz1.^2); % 计算总冲击力

% Torque (Joints)
j_msgs1 = select(bag1, 'Topic', '/franka_state_controller/joint_states');
j_structs1 = readMessages(j_msgs1, 'DataFormat', 'struct');
t_j1 = j_msgs1.MessageList.Time - j_msgs1.MessageList.Time(1);
tau1 = cell2mat(cellfun(@(m) m.Effort(:)', j_structs1, 'UniformOutput', false));
tau_norm1 = sqrt(sum(tau1.^2, 2)); % 计算关节力矩的范数

% --- Dataset 2: Baseline 1 (Without Pre-config) ---
bag2 = rosbag('F:\Home\practice_data\franka_catch_without_pre-optimization_record_with_zmin_02_consider.bag');

% Force
f_msgs2 = select(bag2, 'Topic', '/franka_state_controller/F_ext');
f_structs2 = readMessages(f_msgs2, 'DataFormat', 'struct');
t2 = f_msgs2.MessageList.Time - f_msgs2.MessageList.Time(1);
t2 = t2 - 0.622; % [时间对齐参数]

% 【修正】
Fx2 = cellfun(@(m) m.Wrench.Force.X, f_structs2);
Fy2 = cellfun(@(m) m.Wrench.Force.Y, f_structs2);
Fz2 = cellfun(@(m) m.Wrench.Force.Z, f_structs2);
F_norm2 = sqrt(Fx2.^2 + Fy2.^2 + Fz2.^2);

% Torque
j_msgs2 = select(bag2, 'Topic', '/franka_state_controller/joint_states');
j_structs2 = readMessages(j_msgs2, 'DataFormat', 'struct');
t_j2 = j_msgs2.MessageList.Time - j_msgs2.MessageList.Time(1);
t_j2 = t_j2 - 0.622; % [时间对齐参数]
tau2 = cell2mat(cellfun(@(m) m.Effort(:)', j_structs2, 'UniformOutput', false));
tau_norm2 = sqrt(sum(tau2.^2, 2));

% --- Dataset 3: Baseline 2 (TO Only) ---
bag3 = rosbag('F:\Home\practice_data\franka_catch_with_only_TO_record1.bag');
delta_to = 18.496 - 8.628; % [时间对齐参数]

% Force
f_msgs3 = select(bag3, 'Topic', '/franka_state_controller/F_ext');
f_structs3 = readMessages(f_msgs3, 'DataFormat', 'struct');
t3 = f_msgs3.MessageList.Time - f_msgs3.MessageList.Time(1);
t3 = t3 + delta_to; % [时间对齐参数]

% 【修正】
Fx3 = cellfun(@(m) m.Wrench.Force.X, f_structs3);
Fy3 = cellfun(@(m) m.Wrench.Force.Y, f_structs3);
Fz3 = cellfun(@(m) m.Wrench.Force.Z, f_structs3);
F_norm3 = sqrt(Fx3.^2 + Fy3.^2 + Fz3.^2);

% Torque
j_msgs3 = select(bag3, 'Topic', '/franka_state_controller/joint_states');
j_structs3 = readMessages(j_msgs3, 'DataFormat', 'struct');
t_j3 = j_msgs3.MessageList.Time - j_msgs3.MessageList.Time(1);
t_j3 = t_j3 + delta_to; % [时间对齐参数]
tau3 = cell2mat(cellfun(@(m) m.Effort(:)', j_structs3, 'UniformOutput', false));
tau_norm3 = sqrt(sum(tau3.^2, 2));

% -------------------------------------------------------------------------
% 2. Visualization Settings (期刊风格配置)
% -------------------------------------------------------------------------
% 颜色定义 (Nature/Science 风格)
c_proposed = [0, 0.447, 0.741];      % 深蓝 (Proposed)
c_base1    = [0.850, 0.325, 0.098];  % 砖红 (No Pre-config)
c_base2    = [0.466, 0.674, 0.188];  % 墨绿 (TO Only)
c_gray     = [0.6, 0.6, 0.6];        % 辅助线

% 字体与线宽
font_name = 'Times New Roman';
font_size_label = 12;
font_size_axis = 11;
lw_main = 1.8;
lw_thin = 1.2;

% 创建画布
figure('Units', 'centimeters', 'Position', [5, 5, 18, 14], 'Color', 'w');
t = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% -------------------------------------------------------------------------
% Tile 1: Impact Force Comparison (F_z or F_total)
% -------------------------------------------------------------------------
ax1 = nexttile;
hold on;

% 绘制曲线 (建议画 F_norm 或 Fz，这里以 F_norm 为例，因其代表总冲击)
l1 = plot(t1, F_norm1, '-', 'Color', c_proposed, 'LineWidth', lw_main);
l2 = plot(t2, F_norm2, '--', 'Color', c_base1, 'LineWidth', lw_thin);
l3 = plot(t3, F_norm3, ':', 'Color', c_base2, 'LineWidth', lw_main);

% 添加峰值标注 (自动寻找峰值)
% 假设冲击发生在某个时间窗口，比如 8s 到 12s，根据你的对齐情况调整
% [max_f1, idx1] = max(F_norm1); 
% text(t1(idx1), max_f1, sprintf(' %.1f N', max_f1), 'Color', c_proposed, 'FontSize', 10, 'FontName', font_name);

ylabel('Contact Force $\|\mathbf{F}_{ext}\|$ [N]', 'FontName', font_name, 'FontSize', font_size_label);
grid on; box on;
xlim([1, 2]); % [重要]：根据你的数据范围手动调整X轴视野，聚焦冲击时刻
% ylim(); % 根据数据调整

% 图例
legend([l1, l2, l3],...
    {'\textbf{Ours} (mMMTO + Pre-config)', 'w/o Pre-config', 'TO Only'},...
    'Interpreter', 'latex', 'Location', 'northeast', 'Box', 'off', 'FontSize', 10);

title('\textbf{(a) Impact Force Mitigation}', 'Interpreter', 'latex', 'FontSize', 13);
set(gca, 'FontName', font_name, 'FontSize', font_size_axis, 'LineWidth', 1.0);

% -------------------------------------------------------------------------
% Tile 2: Joint Torque Norm Comparison (Whole-body Effort)
% -------------------------------------------------------------------------
ax2 = nexttile;
hold on;

% 绘制力矩范数 (反映全身受力情况)
p1 = plot(t_j1, tau_norm1, '-', 'Color', c_proposed, 'LineWidth', lw_main);
p2 = plot(t_j2, tau_norm2, '--', 'Color', c_base1, 'LineWidth', lw_thin);
p3 = plot(t_j3, tau_norm3, ':', 'Color', c_base2, 'LineWidth', lw_main);

ylabel('Joint Torque Norm $\|\boldsymbol{\tau}\|$ [Nm]', 'FontName', font_name, 'FontSize', font_size_label);
xlabel('Time [s]', 'FontName', font_name, 'FontSize', font_size_label);
grid on; box on;
xlim([1, 2]); % 保持与上面对齐

% 标注关键差异区域 (例如冲击时的力矩峰值)
% 你可以用 patch 函数在背景画一个淡淡的灰色区域表示 "Impact Phase"
% x_start = 10.5; x_end = 11.0; % 示例时间
% y_lim = ylim;
% patch([x_start x_end x_end x_start], [y_lim(1) y_lim(1) y_lim(2) y_lim(2)],...
%       [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
% text((x_start+x_end)/2, y_lim(2)*0.9, 'Impact', 'HorizontalAlignment', 'center', 'FontSize', 9);

title('\textbf{(b) Whole-body Torque Distribution}', 'Interpreter', 'latex', 'FontSize', 13);
set(gca, 'FontName', font_name, 'FontSize', font_size_axis, 'LineWidth', 1.0);

% -------------------------------------------------------------------------
% Global Adjustments
% -------------------------------------------------------------------------
linkaxes([ax1, ax2], 'x'); % 联动X轴缩放

% 导出设置
% exportgraphics(gcf, 'Experimental_Comparison.pdf', 'ContentType', 'vector');