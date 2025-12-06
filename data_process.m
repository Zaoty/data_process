% filepath: /home/q1ren/matlab_ws/catching/catching_brick_data_full_opti.m
% 解包 rosbag 文件并读取各 topic 的数据
clear all
bag = rosbag('/home/q1ren/practice_data/franka_catch_record_full_opti9_deside.bag');
% bag = rosbag('/home/q1ren/franka_catch_full_opti_record1.bag');

% 读取各 topic 的消息
topics = { ...
    '/franka_state_controller/F_ext', ...
    '/franka_state_controller/franka_states', ...
    '/franka_state_controller/joint_states', ...
    '/odom', ...
    '/joint_states', ...
    '/gazebo/link_states', ...
    '/gazebo/model_states', ...
    '/cmd_vel' ...
};

for i = 1:length(topics)
    topic = topics{i};
    disp(['Reading topic: ', topic]);
    msgs = select(bag, 'Topic', topic);
    % 可根据需要进一步处理 msgs
    disp(['Number of messages: ', num2str(msgs.NumMessages)]);
end

% --- 设置绘图风格 (Journal Style) ---
set(0, 'DefaultAxesFontName', 'Computer Modern'); % 修改此处：使用 Computer Modern 字体
set(0, 'DefaultAxesFontSize', 22); % 增大默认坐标轴字号
set(0, 'DefaultTextFontSize', 22); % 增大默认文本字号
set(0, 'DefaultLineLineWidth', 2);
set(0, 'DefaultFigureColor', 'w');
set(0, 'DefaultTextInterpreter', 'latex'); % 新增：全局使用 latex 解释器
set(0, 'DefaultLegendInterpreter', 'latex'); % 新增：图例使用 latex
set(0, 'DefaultAxesTickLabelInterpreter', 'latex'); % 新增：坐标轴标签使用 latex
% 定义配色方案 (Red, Green, Blue) - 红色降低饱和度，蓝色突出显示
c_red = [0.8500 0.5500 0.5500];   % #D98C8C (Muted Red)
c_green = [0.4500 0.7200 0.7200]; % #73B8B8 (Unchanged)
c_blue = [0.1000 0.3500 0.8500];  % #1A59D9 (Vivid Blue)
% -----------------------------------

% 1. 末端受力和力矩（/franka_state_controller/F_ext）
fext_msgs = select(bag, 'Topic', '/franka_state_controller/F_ext');
fext_structs = readMessages(fext_msgs, 'DataFormat', 'struct');
fext_time = fext_msgs.MessageList.Time - fext_msgs.MessageList.Time(1);

Fx = cellfun(@(m) m.Wrench.Force.X, fext_structs);
Fy = cellfun(@(m) m.Wrench.Force.Y, fext_structs);
Fz = cellfun(@(m) m.Wrench.Force.Z, fext_structs);
Tx = cellfun(@(m) m.Wrench.Torque.X, fext_structs);
Ty = cellfun(@(m) m.Wrench.Torque.Y, fext_structs);
Tz = cellfun(@(m) m.Wrench.Torque.Z, fext_structs);

% [Deleted Figure 1: End-effector Force and Torque]

% 2. 各关节力矩（/franka_state_controller/joint_states）
joint_msgs = select(bag, 'Topic', '/franka_state_controller/joint_states');
joint_structs = readMessages(joint_msgs, 'DataFormat', 'struct');
joint_time = joint_msgs.MessageList.Time - joint_msgs.MessageList.Time(1);
efforts = cell2mat(cellfun(@(m) m.Effort(:)', joint_structs, 'UniformOutput', false));

% [Deleted Figure 2: Joint Torques]

% 读取第二个 rosbag 文件
bag2 = rosbag('/home/q1ren/practice_data/franka_catch_record_without_pre_opti1.bag');
% bag2 = rosbag('/home/q1ren/franka_catch_without_pre-configuration_record1.bag');

% 提取第二个文件的关节力矩
joint_msgs2 = select(bag2, 'Topic', '/franka_state_controller/joint_states');
joint_structs2 = readMessages(joint_msgs2, 'DataFormat', 'struct');
joint_time2 = joint_msgs2.MessageList.Time - joint_msgs2.MessageList.Time(1);
joint_time2 = joint_time2 - 0.622; % 时间轴前移3.911秒

efforts2 = cell2mat(cellfun(@(m) m.Effort(:)', joint_structs2, 'UniformOutput', false));
% 调试：检查两个数据源的幅值
disp(['bag1 (With Pre-config) Max Torque: ', num2str(max(abs(efforts(:))))]);
disp(['bag2 (No Pre-config) Max Torque: ', num2str(max(abs(efforts2(:))))]);

% [Deleted Figure 3: Joint Torque Comparison (2 Sets)]

% 提取第二个文件的末端受力和力矩
fext_msgs2 = select(bag2, 'Topic', '/franka_state_controller/F_ext');
fext_structs2 = readMessages(fext_msgs2, 'DataFormat', 'struct');
fext_time2 = fext_msgs2.MessageList.Time - fext_msgs2.MessageList.Time(1);
fext_time2 = fext_time2 - 0.622; % 时间轴前移3.911秒

Fx2 = cellfun(@(m) m.Wrench.Force.X, fext_structs2);
Fy2 = cellfun(@(m) m.Wrench.Force.Y, fext_structs2);
Fz2 = cellfun(@(m) m.Wrench.Force.Z, fext_structs2);
Tx2 = cellfun(@(m) m.Wrench.Torque.X, fext_structs2);
Ty2 = cellfun(@(m) m.Wrench.Torque.Y, fext_structs2);
Tz2 = cellfun(@(m) m.Wrench.Torque.Z, fext_structs2);

% [Deleted Figure 4: End-effector Force/Torque Comparison (2 Sets)]

% 计算总受力和总力矩
F_total = sqrt(Fx.^2 + Fy.^2 + Fz.^2);
F_total2 = sqrt(Fx2.^2 + Fy2.^2 + Fz2.^2);
T_total = sqrt(Tx.^2 + Ty.^2 + Tz.^2);
T_total2 = sqrt(Tx2.^2 + Ty2.^2 + Tz2.^2);

% [Deleted Figure 5: Force Component Comparison (2 Sets)]

% [Deleted Figure 6: Torque Component Comparison (2 Sets)]

% 读取第三个 rosbag 文件
% bag3 = rosbag('F:\Home\practice_data\franka_catch_with_only_TO_record1.bag');
bag3 = rosbag('/home/q1ren/practice_data/franka_catch_with_only_TO_record1.bag');
delta_to = 18.496 - 8.628; % 第三个文件比第一个文件早的时间差

% 提取第三个文件的关节力矩
joint_msgs3 = select(bag3, 'Topic', '/franka_state_controller/joint_states');
joint_structs3 = readMessages(joint_msgs3, 'DataFormat', 'struct');
joint_time3 = joint_msgs3.MessageList.Time - joint_msgs3.MessageList.Time(1);
joint_time3 = joint_time3 + delta_to; % 时间轴后移对齐
efforts3 = cell2mat(cellfun(@(m) m.Effort(:)', joint_structs3, 'UniformOutput', false));

disp(['bag3 (TO Only) Max Torque: ', num2str(max(abs(efforts3(:))))]);

%{
% 对比绘图（关节力矩 - 三个文件）
num_joints = max([size(efforts,2), size(efforts2,2), size(efforts3,2)]);
figure('Name', 'Joint Torque Comparison (3 Sets)', 'Position', [100, 100, 1000, 800]);
for j = 1:num_joints
    subplot(4,2,j);
    has1 = size(efforts,2) >= j;
    has2 = size(efforts2,2) >= j;
    has3 = size(efforts3,2) >= j;
    plotted = false;
    if has1 && ~isempty(efforts(:,j))
        plot(joint_time, efforts(:,j), 'Color', c_blue, 'DisplayName', 'With Pre-config');
        plotted = true;
        hold on;
    end
    if has2 && ~isempty(efforts2(:,j))
        plot(joint_time2, efforts2(:,j), 'Color', c_red, 'LineStyle', '-', 'DisplayName', 'Without Pre-config');
        plotted = true;
    end
    if has3 && ~isempty(efforts3(:,j))
        plot(joint_time3, efforts3(:,j), 'Color', c_green, 'LineStyle', '-', 'DisplayName', 'TO Only');
        plotted = true;
    end
    if plotted
        hold off;
        title(['Joint ', num2str(j), ' Torque Comparison'], 'FontWeight', 'bold');
        xlabel('Time (s)');
        ylabel('Torque (Nm)');
        if j == 1, legend('Location', 'best', 'Box', 'off'); end
        grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;
    else
        title(['Joint ', num2str(j), ' No Data']);
    end
end
%}

% 提取第三个文件的末端受力和力矩
fext_msgs3 = select(bag3, 'Topic', '/franka_state_controller/F_ext');
fext_structs3 = readMessages(fext_msgs3, 'DataFormat', 'struct');
fext_time3 = fext_msgs3.MessageList.Time - fext_msgs3.MessageList.Time(1);
fext_time3 = fext_time3 + delta_to;

Fx3 = cellfun(@(m) m.Wrench.Force.X, fext_structs3);
Fy3 = cellfun(@(m) m.Wrench.Force.Y, fext_structs3);
Fz3 = cellfun(@(m) m.Wrench.Force.Z, fext_structs3);
Tx3 = cellfun(@(m) m.Wrench.Torque.X, fext_structs3);
Ty3 = cellfun(@(m) m.Wrench.Torque.Y, fext_structs3);
Tz3 = cellfun(@(m) m.Wrench.Torque.Z, fext_structs3);

%{
% 对比末端受力（三个文件）
figure('Name', 'End-effector Force Comparison (3 Sets)');
subplot(2,1,1);
plot(fext_time, Fx, 'Color', c_blue, 'DisplayName', 'F_x With Pre-config'); hold on;
plot(fext_time2, Fx2, 'Color', c_blue, 'LineStyle', '-', 'DisplayName', 'F_x Without Pre-config');
plot(fext_time3, Fx3, 'Color', c_blue, 'LineStyle', '-', 'DisplayName', 'F_x TO Only');

plot(fext_time, Fy, 'Color', c_red, 'DisplayName', 'F_y With Pre-config');
plot(fext_time2, Fy2, 'Color', c_red, 'LineStyle', '-', 'DisplayName', 'F_y Without Pre-config');
plot(fext_time3, Fy3, 'Color', c_red, 'LineStyle', '-', 'DisplayName', 'F_y TO Only');

plot(fext_time, Fz, 'Color', c_green, 'DisplayName', 'F_z With Pre-config');
plot(fext_time2, Fz2, 'Color', c_green, 'LineStyle', '-', 'DisplayName', 'F_z Without Pre-config');
plot(fext_time3, Fz3, 'Color', c_green, 'LineStyle', '-', 'DisplayName', 'F_z TO Only');
hold off;
title('End-effector Force Comparison', 'FontWeight', 'bold');
xlabel('Time (s)');
ylabel('Force (N)');
legend('NumColumns', 3, 'Location', 'best', 'Box', 'off');
grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

% 对比末端力矩（三个文件）
subplot(2,1,2);
plot(fext_time, Tx, 'Color', c_blue, 'DisplayName', 'T_x With Pre-config'); hold on;
plot(fext_time2, Tx2, 'Color', c_blue, 'LineStyle', '-', 'DisplayName', 'T_x Without Pre-config');
plot(fext_time3, Tx3, 'Color', c_blue, 'LineStyle', '-', 'DisplayName', 'T_x TO Only');

plot(fext_time, Ty, 'Color', c_red, 'DisplayName', 'T_y With Pre-config');
plot(fext_time2, Ty2, 'Color', c_red, 'LineStyle', '-', 'DisplayName', 'T_y Without Pre-config');
plot(fext_time3, Ty3, 'Color', c_red, 'LineStyle', '-', 'DisplayName', 'T_y TO Only');

plot(fext_time, Tz, 'Color', c_green, 'DisplayName', 'T_z With Pre-config');
plot(fext_time2, Tz2, 'Color', c_green, 'LineStyle', '-', 'DisplayName', 'T_z Without Pre-config');
plot(fext_time3, Tz3, 'Color', c_green, 'LineStyle', '-', 'DisplayName', 'T_z TO Only');
hold off;
title('End-effector Torque Comparison', 'FontWeight', 'bold');
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('NumColumns', 3, 'Location', 'best', 'Box', 'off');
grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;
%}

% 计算第三个文件的总受力和总力矩
F_total3 = sqrt(Fx3.^2 + Fy3.^2 + Fz3.^2);
T_total3 = sqrt(Tx3.^2 + Ty3.^2 + Tz3.^2);

%{
% 分量和总受力对比（三个文件）
figure('Name', 'Force Component Comparison (3 Sets)', 'Position', [100, 100, 800, 800]);
subplot(4,1,1);
plot(fext_time, Fx, 'Color', c_blue); hold on;
plot(fext_time2, Fx2, 'Color', c_red, 'LineStyle', '-');
plot(fext_time3, Fx3, 'Color', c_green, 'LineStyle', '-'); hold off;
title('F_x Component Comparison', 'FontWeight', 'bold'); ylabel('F_x (N)'); legend('With Pre-config','Without Pre-config','TO Only', 'Box', 'off'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,2);
plot(fext_time, Fy, 'Color', c_blue); hold on;
plot(fext_time2, Fy2, 'Color', c_red, 'LineStyle', '-');
plot(fext_time3, Fy3, 'Color', c_green, 'LineStyle', '-'); hold off;
title('F_y Component Comparison', 'FontWeight', 'bold'); ylabel('F_y (N)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,3);
plot(fext_time, Fz, 'Color', c_blue); hold on;
plot(fext_time2, Fz2, 'Color', c_red, 'LineStyle', '-');
plot(fext_time3, Fz3, 'Color', c_green, 'LineStyle', '-'); hold off;
title('F_z Component Comparison', 'FontWeight', 'bold'); ylabel('F_z (N)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,4);
plot(fext_time, F_total, 'Color', c_blue); hold on;
plot(fext_time2, F_total2, 'Color', c_red, 'LineStyle', '-');
plot(fext_time3, F_total3, 'Color', c_green, 'LineStyle', '-'); hold off;
title('Total Force Comparison', 'FontWeight', 'bold'); xlabel('Time (s)'); ylabel('Total Force (N)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

% 分量和总力矩对比（三个文件）
figure('Name', 'Torque Component Comparison (3 Sets)', 'Position', [100, 100, 800, 800]);
subplot(4,1,1);
plot(fext_time, Tx, 'Color', c_blue); hold on;
plot(fext_time2, Tx2, 'Color', c_red, 'LineStyle', '-');
plot(fext_time3, Tx3, 'Color', c_green, 'LineStyle', '-'); hold off;
title('T_x Component Comparison', 'FontWeight', 'bold'); ylabel('T_x (Nm)'); legend('With Pre-config','Without Pre-config','TO Only', 'Box', 'off'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,2);
plot(fext_time, Ty, 'Color', c_blue); hold on;
plot(fext_time2, Ty2, 'Color', c_red, 'LineStyle', '-');
plot(fext_time3, Ty3, 'Color', c_green, 'LineStyle', '-'); hold off;
title('T_y Component Comparison', 'FontWeight', 'bold'); ylabel('T_y (Nm)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,3);
plot(fext_time, Tz, 'Color', c_blue); hold on;
plot(fext_time2, Tz2, 'Color', c_red, 'LineStyle', '-');
plot(fext_time3, Tz3, 'Color', c_green, 'LineStyle', '-'); hold off;
title('T_z Component Comparison', 'FontWeight', 'bold'); ylabel('T_z (Nm)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,4);
plot(fext_time, T_total, 'Color', c_blue); hold on;
plot(fext_time2, T_total2, 'Color', c_red, 'LineStyle', '-');
plot(fext_time3, T_total3, 'Color', c_green, 'LineStyle', '-'); hold off;
title('Total Torque Comparison', 'FontWeight', 'bold'); xlabel('Time (s)'); ylabel('Total Torque (Nm)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;
%}

%{
% --- 新增：关节力矩极坐标图 (Polar Plots for Torque Analysis) ---
% 计算指标
max_tau1 = max(abs(efforts), [], 1);
max_tau2 = max(abs(efforts2), [], 1);
max_tau3 = max(abs(efforts3), [], 1);

rms_tau1 = rms(efforts, 1);
rms_tau2 = rms(efforts2, 1);
rms_tau3 = rms(efforts3, 1);

tau_lim = [87, 87, 87, 87, 12, 12, 12]; % 关节力矩限制

% 准备极坐标数据 (闭合回路)
n_joints = 7;
angles = linspace(0, 2*pi, n_joints+1);
close_loop = @(x) [x(:)', x(1)]; % 辅助函数：闭合数据

% 辅助函数：极坐标转笛卡尔坐标
pol2cart_custom = @(rho, theta) [rho .* cos(theta); rho .* sin(theta)];

figure('Name', 'Torque Polar Analysis', 'Position', [100, 100, 1200, 600]);

% 子图 1: 最大力矩
subplot(1, 2, 1);
hold on; axis equal; axis off;
max_r_left = 87;
ticks_left = 10:10:80; % 修改：最外圈87

% 绘制网格圆
theta_grid = linspace(0, 2*pi, 200);
plot(max_r_left * cos(theta_grid), max_r_left * sin(theta_grid), 'k-', 'LineWidth', 1.5); % 边界
for r = ticks_left
    plot(r * cos(theta_grid), r * sin(theta_grid), 'Color', [0.8 0.8 0.8], 'LineWidth', 1.5);
end

% 绘制网格线 (Spokes)
for i = 1:n_joints
    plot([0, max_r_left * cos(angles(i))], [0, max_r_left * sin(angles(i))], 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
    % 绘制标签 (tau1, tau2...)
    label_r = max_r_left * 1.15;
    text(label_r * cos(angles(i)), label_r * sin(angles(i)), ['$\tau_', num2str(i), '$'], ...
        'HorizontalAlignment', 'center', 'FontSize', 35, 'FontName', 'Computer Modern', 'Interpreter', 'latex');
end

% 绘制数据 (填充 + 实线 + 标记)
% 计算坐标
xy1 = pol2cart_custom(close_loop(max_tau1), angles);
xy2 = pol2cart_custom(close_loop(max_tau2), angles);
xy3 = pol2cart_custom(close_loop(max_tau3), angles);

% 创建用于图例的 Dummy Plot (线+点)，不显示在图中(NaN)
L1 = plot(nan, nan, '-o', 'Color', c_blue, 'LineWidth', 2, 'MarkerFaceColor', c_blue, 'DisplayName', 'With Pre-config');
L2 = plot(nan, nan, '-o', 'Color', c_red, 'LineWidth', 2, 'MarkerFaceColor', c_red, 'DisplayName', 'Without Pre-config');
L3 = plot(nan, nan, '-o', 'Color', c_green, 'LineWidth', 2, 'MarkerFaceColor', c_green, 'DisplayName', 'TO Only');

% 按图例顺序从上到下对应绘制顺序从底到顶 (TO Only -> Without -> With)
% TO Only (Green/Teal) - Bottom Layer
fill(xy3(1,:), xy3(2,:), c_green, 'FaceAlpha', 0.2, 'EdgeColor', c_green, 'LineWidth', 3, ...
    'Marker', 'o', 'MarkerFaceColor', c_green);
% Without Pre-config (Red/Orange) - Middle Layer
fill(xy2(1,:), xy2(2,:), c_red, 'FaceAlpha', 0.2, 'EdgeColor', c_red, 'LineWidth', 3, ...
    'Marker', 'o', 'MarkerFaceColor', c_red);
% With Pre-config (Blue/Purple) - Top Layer
fill(xy1(1,:), xy1(2,:), c_blue, 'FaceAlpha', 0.2, 'EdgeColor', c_blue, 'LineWidth', 3, ...
    'Marker', 'o', 'MarkerFaceColor', c_blue);

% 绘制限制 (J5-J7: 12Nm)
theta_lim_arc = linspace(angles(5), angles(7), 100);
plot(12 * cos(theta_lim_arc), 12 * sin(theta_lim_arc), 'k-', 'LineWidth', 2);

% 标题
% title('$\tau_{max}$', 'FontWeight', 'bold', 'FontSize', 50, 'Position', [0, max_r_left*1.3, 0]);
text(0, max_r_left*1.3, '$\tau_{max}$', 'FontWeight', 'bold', 'FontSize', 50, ...
    'HorizontalAlignment', 'center', 'Interpreter', 'latex');

% 刻度数值
angle_offset = deg2rad(5);
for i = 1:length(ticks_left)
    r_val = ticks_left(i);
    % 只在偶数圈标注数值，且不标注80
    if mod(i, 2) == 0 
        txt = num2str(r_val);
        % if r_val == max(ticks_left)
        %     txt = [txt, ' Nm'];
        % end
        text(r_val * cos(angle_offset), r_val * sin(angle_offset), txt, ...
            'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 20, 'FontName', 'Computer Modern', 'Interpreter', 'latex');
    end
end

% 手动添加 87 Nm 和 tau_lim
text(max_r_left * cos(angle_offset), max_r_left * sin(angle_offset), '87 Nm', ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 20, 'FontName', 'Computer Modern', 'Interpreter', 'latex');
text(max_r_left * cos(angle_offset), max_r_left * sin(angle_offset) + 5, '$\tau_{lim}$', ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 25, 'FontName', 'Computer Modern', 'Interpreter', 'latex');

% 限制标注
text(12 * cos(angles(6)), 12 * sin(angles(6)) + 5, '$\tau_{lim}$', ...
    'HorizontalAlignment', 'center', 'FontSize', 28, 'FontWeight', 'bold', 'Color', 'k', 'FontName', 'Computer Modern', 'Interpreter', 'latex');


% 子图 2: RMS 力矩
subplot(1, 2, 2);
hold on; axis equal; axis off;
max_r_right = 30;
ticks_right = 5:5:30;

% 绘制网格圆
plot(max_r_right * cos(theta_grid), max_r_right * sin(theta_grid), 'k-', 'LineWidth', 1.5); % 边界
for r = ticks_right
    plot(r * cos(theta_grid), r * sin(theta_grid), 'Color', [0.8 0.8 0.8], 'LineWidth', 1.5);
end

% 绘制网格线 (Spokes)
for i = 1:n_joints
    plot([0, max_r_right * cos(angles(i))], [0, max_r_right * sin(angles(i))], 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
    % 绘制标签
    label_r = max_r_right * 1.15;
    text(label_r * cos(angles(i)), label_r * sin(angles(i)), ['$\tau_', num2str(i), '$'], ...
        'HorizontalAlignment', 'center', 'FontSize', 35, 'FontName', 'Computer Modern', 'Interpreter', 'latex');
end

% 绘制数据 (填充 + 实线 + 标记)
% 计算坐标
xy1_rms = pol2cart_custom(close_loop(rms_tau1), angles);
xy2_rms = pol2cart_custom(close_loop(rms_tau2), angles);
xy3_rms = pol2cart_custom(close_loop(rms_tau3), angles);

% 按图例顺序从上到下对应绘制顺序从底到顶 (TO Only -> Without -> With)
% TO Only (Green/Teal)
fill(xy3_rms(1,:), xy3_rms(2,:), c_green, 'FaceAlpha', 0.2, 'EdgeColor', c_green, 'LineWidth', 3, ...
    'Marker', 'o', 'MarkerFaceColor', c_green);
% Without Pre-config (Red/Orange)
fill(xy2_rms(1,:), xy2_rms(2,:), c_red, 'FaceAlpha', 0.2, 'EdgeColor', c_red, 'LineWidth', 3, ...
    'Marker', 'o', 'MarkerFaceColor', c_red);
% With Pre-config (Blue/Purple)
fill(xy1_rms(1,:), xy1_rms(2,:), c_blue, 'FaceAlpha', 0.2, 'EdgeColor', c_blue, 'LineWidth', 3, ...
    'Marker', 'o', 'MarkerFaceColor', c_blue);

% 绘制限制 (J5-J7: 12Nm)
plot(12 * cos(theta_lim_arc), 12 * sin(theta_lim_arc), 'k-', 'LineWidth', 2);

% 标题
% title('$\tau_{RMS}$', 'FontWeight', 'bold', 'FontSize', 50, 'Position', [0, max_r_right*1.3, 0]);
text(0, max_r_right*1.3, '$\tau_{RMS}$', 'FontWeight', 'bold', 'FontSize', 50, ...
    'HorizontalAlignment', 'center', 'Interpreter', 'latex');

% 刻度数值
for i = 1:length(ticks_right)
    r_val = ticks_right(i);
    % 只在偶数圈标注数值
    if mod(i, 2) == 0
        txt = num2str(r_val);
        if r_val == max(ticks_right)
            txt = [txt, ' Nm'];
        end
        text(r_val * cos(angle_offset), r_val * sin(angle_offset), txt, ...
            'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 20, 'FontName', 'Computer Modern', 'Interpreter', 'latex');
    end
end

% 限制标注
text(12 * cos(angles(6)), 12 * sin(angles(6)) + 2, '$\tau_{lim}$', ...
    'HorizontalAlignment', 'center', 'FontSize', 28, 'FontWeight', 'bold', 'Color', 'k', 'FontName', 'Computer Modern', 'Interpreter', 'latex');

% 统一图例 (放在两图中间下方，竖直排列)
% 顺序颠倒：TO Only, Without, With
lgd = legend([L3, L2, L1], '$MMTO$', '$mMMTO-DIM$', '$mMMTO-DIM-PGO$', ...
    'Box', 'on', 'NumColumns', 1, 'FontSize', 25);
% lgd = legend([L2, L1], '$mMMTO-DIM$', '$mMMTO-DIM-PGO$', ...
%     'Box', 'on', 'NumColumns', 1, 'FontSize', 25);
% 手动调整图例位置到 Figure 底部居中
lgd.Position(1) = 0.5 - lgd.Position(3)/2; % 水平居中
lgd.Position(2) = 0.02; % 底部
%}

% --- 新增：仿照图表绘制 (Comparison Plot) ---
disp('正在提取 Link 和 Model States 数据以绘制对比图...');

% 定义时间偏移 (将关键时刻设为 0)
t_offset1 = 18.407-0.136;
t_offset2 = 17.844+0.614-0.948-0.229;
t_offset3 = 8.483;

% 定义偏移量
ee_offset = 0.01/2;   % plate 上表面
obj_offset = -0.050/2; % stone 下表面

% --- Bag 1 (With Pre-config) ---
disp('Processing Bag 1 Link/Model...');
link_msgs1 = select(bag, 'Topic', '/gazebo/link_states');
link_structs1 = readMessages(link_msgs1, 'DataFormat', 'struct');
link_time1 = link_msgs1.MessageList.Time - link_msgs1.MessageList.Time(1) - t_offset1;

link_names = link_structs1{1}.Name;
ee_idx = find(strcmp(link_names, 'panda::panda_plate'), 1);
ee_z1 = cellfun(@(m) m.Pose(ee_idx).Position.Z, link_structs1) + ee_offset;
ee_vz1 = cellfun(@(m) m.Twist(ee_idx).Linear.Z, link_structs1);

model_msgs1 = select(bag, 'Topic', '/gazebo/model_states');
model_structs1 = readMessages(model_msgs1, 'DataFormat', 'struct');
model_time1 = model_msgs1.MessageList.Time - model_msgs1.MessageList.Time(1) - t_offset1;

model_names = model_structs1{1}.Name;
obj_idx = find(strcmp(model_names, 'stone'), 1);
obj_z1 = cellfun(@(m) m.Pose(obj_idx).Position.Z, model_structs1) + obj_offset;
obj_vz1 = cellfun(@(m) m.Twist(obj_idx).Linear.Z, model_structs1);

% --- Bag 2 (Without Pre-config) ---
disp('Processing Bag 2 Link/Model...');
link_msgs2 = select(bag2, 'Topic', '/gazebo/link_states');
link_structs2 = readMessages(link_msgs2, 'DataFormat', 'struct');
link_time2 = link_msgs2.MessageList.Time - link_msgs2.MessageList.Time(1) - t_offset2;

ee_z2 = cellfun(@(m) m.Pose(ee_idx).Position.Z, link_structs2) + ee_offset;
ee_vz2 = cellfun(@(m) m.Twist(ee_idx).Linear.Z, link_structs2);

model_msgs2 = select(bag2, 'Topic', '/gazebo/model_states');
model_structs2 = readMessages(model_msgs2, 'DataFormat', 'struct');
model_time2 = model_msgs2.MessageList.Time - model_msgs2.MessageList.Time(1) - t_offset2;

obj_z2 = cellfun(@(m) m.Pose(obj_idx).Position.Z, model_structs2) + obj_offset;
obj_vz2 = cellfun(@(m) m.Twist(obj_idx).Linear.Z, model_structs2);

% --- Bag 3 (TO Only) ---
disp('Processing Bag 3 Link/Model...');
link_msgs3 = select(bag3, 'Topic', '/gazebo/link_states');
link_structs3 = readMessages(link_msgs3, 'DataFormat', 'struct');
link_time3 = link_msgs3.MessageList.Time - link_msgs3.MessageList.Time(1) - t_offset3;

ee_z3 = cellfun(@(m) m.Pose(ee_idx).Position.Z, link_structs3) + ee_offset;
ee_vz3 = cellfun(@(m) m.Twist(ee_idx).Linear.Z, link_structs3);

model_msgs3 = select(bag3, 'Topic', '/gazebo/model_states');
model_structs3 = readMessages(model_msgs3, 'DataFormat', 'struct');
model_time3 = model_msgs3.MessageList.Time - model_msgs3.MessageList.Time(1) - t_offset3;

obj_z3 = cellfun(@(m) m.Pose(obj_idx).Position.Z, model_structs3) + obj_offset;
obj_vz3 = cellfun(@(m) m.Twist(obj_idx).Linear.Z, model_structs3);

% --- 计算 Fext 的绘图时间 ---
fext_time_plot1 = fext_msgs.MessageList.Time - fext_msgs.MessageList.Time(1) - t_offset1;
fext_time_plot2 = fext_msgs2.MessageList.Time - fext_msgs2.MessageList.Time(1) - t_offset2;
fext_time_plot3 = fext_msgs3.MessageList.Time - fext_msgs3.MessageList.Time(1) - t_offset3;

% --- 数据截取 (0s - 5s) ---
t_min = -1;
t_max = 3;

% Bag 1
idx_l1 = link_time1 >= t_min & link_time1 <= t_max;
link_time1 = link_time1(idx_l1); ee_z1 = ee_z1(idx_l1); ee_vz1 = ee_vz1(idx_l1);

idx_m1 = model_time1 >= t_min & model_time1 <= t_max;
model_time1 = model_time1(idx_m1); obj_z1 = obj_z1(idx_m1); obj_vz1 = obj_vz1(idx_m1);

idx_f1 = fext_time_plot1 >= t_min & fext_time_plot1 <= t_max;
fext_time_plot1 = fext_time_plot1(idx_f1); Fz = Fz(idx_f1); F_total = F_total(idx_f1);

% Bag 2
idx_l2 = link_time2 >= t_min & link_time2 <= t_max;
link_time2 = link_time2(idx_l2); ee_z2 = ee_z2(idx_l2); ee_vz2 = ee_vz2(idx_l2);

idx_m2 = model_time2 >= t_min & model_time2 <= t_max;
model_time2 = model_time2(idx_m2); obj_z2 = obj_z2(idx_m2); obj_vz2 = obj_vz2(idx_m2);

idx_f2 = fext_time_plot2 >= t_min & fext_time_plot2 <= t_max;
fext_time_plot2 = fext_time_plot2(idx_f2); Fz2 = Fz2(idx_f2); F_total2 = F_total2(idx_f2);

% Bag 3
idx_l3 = link_time3 >= t_min & link_time3 <= t_max;
link_time3 = link_time3(idx_l3); ee_z3 = ee_z3(idx_l3); ee_vz3 = ee_vz3(idx_l3);

idx_m3 = model_time3 >= t_min & model_time3 <= t_max;
model_time3 = model_time3(idx_m3); obj_z3 = obj_z3(idx_m3); obj_vz3 = obj_vz3(idx_m3);

idx_f3 = fext_time_plot3 >= t_min & fext_time_plot3 <= t_max;
fext_time_plot3 = fext_time_plot3(idx_f3); Fz3 = Fz3(idx_f3); F_total3 = F_total3(idx_f3);

% --- 绘图 ---
figure('Name', 'Experiment Comparison', 'Position', [50, 50, 1400, 900], 'Color', 'w');

% Row 1: With Pre-config (Blue)
subplot(3, 4, 1);
plot(link_time1, ee_z1, 'Color', c_blue, 'LineWidth', 2, 'DisplayName', 'Robot $z$'); hold on;
plot(model_time1, obj_z1, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Object $z$');
ylabel({'With Pre-config', '$z [m]$'}, 'Interpreter', 'latex');
title('$z [m]$', 'Interpreter', 'latex');
grid on; set(gca, 'GridAlpha', 0.3); axis tight;
legend('Location', 'best', 'Box', 'off', 'Interpreter', 'latex');

subplot(3, 4, 2);
plot(link_time1, ee_vz1, 'Color', c_blue, 'LineWidth', 2, 'DisplayName', 'Robot $\dot{z}$'); hold on;
plot(model_time1, obj_vz1, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Object $\dot{z}$');
title('$\dot{z} [m/s]$', 'Interpreter', 'latex');
grid on; set(gca, 'GridAlpha', 0.3); axis tight;
legend('Location', 'best', 'Box', 'off', 'Interpreter', 'latex');

subplot(3, 4, 3);
plot(fext_time_plot1, Fz, 'Color', 'k', 'LineWidth', 1.5, 'DisplayName', '$F_z$');
title('$F_z [N]$', 'Interpreter', 'latex');
grid on; set(gca, 'GridAlpha', 0.3); axis tight;

subplot(3, 4, 4);
plot(fext_time_plot1, F_total, 'Color', 'k', 'LineWidth', 1.5, 'DisplayName', '$F_{total}$');
title('$F_{total} [N]$', 'Interpreter', 'latex');
grid on; set(gca, 'GridAlpha', 0.3); axis tight;

% Row 2: Without Pre-config (Red)
subplot(3, 4, 5);
plot(link_time2, ee_z2, 'Color', c_red, 'LineWidth', 2, 'DisplayName', 'Robot $z$'); hold on;
plot(model_time2, obj_z2, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Object $z$');
ylabel({'Without Pre-config', '$z [m]$'}, 'Interpreter', 'latex');
grid on; set(gca, 'GridAlpha', 0.3); axis tight;
legend('Location', 'best', 'Box', 'off', 'Interpreter', 'latex');

subplot(3, 4, 6);
plot(link_time2, ee_vz2, 'Color', c_red, 'LineWidth', 2, 'DisplayName', 'Robot $\dot{z}$'); hold on;
plot(model_time2, obj_vz2, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Object $\dot{z}$');
grid on; set(gca, 'GridAlpha', 0.3); axis tight;
legend('Location', 'best', 'Box', 'off', 'Interpreter', 'latex');

subplot(3, 4, 7);
plot(fext_time_plot2, Fz2, 'Color', 'k', 'LineWidth', 1.5, 'DisplayName', '$F_z$');
grid on; set(gca, 'GridAlpha', 0.3); axis tight;

subplot(3, 4, 8);
plot(fext_time_plot2, F_total2, 'Color', 'k', 'LineWidth', 1.5, 'DisplayName', '$F_{total}$');
grid on; set(gca, 'GridAlpha', 0.3); axis tight;

% Row 3: TO Only (Green)
subplot(3, 4, 9);
plot(link_time3, ee_z3, 'Color', c_green, 'LineWidth', 2, 'DisplayName', 'Robot $z$'); hold on;
plot(model_time3, obj_z3, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Object $z$');
ylabel({'TO Only', '$z [m]$'}, 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
grid on; set(gca, 'GridAlpha', 0.3); axis tight;
legend('Location', 'best', 'Box', 'off', 'Interpreter', 'latex');

subplot(3, 4, 10);
plot(link_time3, ee_vz3, 'Color', c_green, 'LineWidth', 2, 'DisplayName', 'Robot $\dot{z}$'); hold on;
plot(model_time3, obj_vz3, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Object $\dot{z}$');
xlabel('Time [s]', 'Interpreter', 'latex');
grid on; set(gca, 'GridAlpha', 0.3); axis tight;
legend('Location', 'best', 'Box', 'off', 'Interpreter', 'latex');

subplot(3, 4, 11);
plot(fext_time_plot3, Fz3, 'Color', 'k', 'LineWidth', 1.5, 'DisplayName', '$F_z$');
xlabel('Time [s]', 'Interpreter', 'latex');
grid on; set(gca, 'GridAlpha', 0.3); axis tight;

subplot(3, 4, 12);
plot(fext_time_plot3, F_total3, 'Color', 'k', 'LineWidth', 1.5, 'DisplayName', '$F_{total}$');
xlabel('Time [s]', 'Interpreter', 'latex');
grid on; set(gca, 'GridAlpha', 0.3); axis tight;