% filepath: /home/q1ren/matlab_ws/catching/catching_brick_data_full_opti.m
% 解包 rosbag 文件并读取各 topic 的数据
clear all
bag = rosbag('F:\Home\practice_data\franka_catch_full_opti_record_consider.bag');
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
    disp(['读取 topic: ', topic]);
    msgs = select(bag, 'Topic', topic);
    % 可根据需要进一步处理 msgs
    disp(['消息数量: ', num2str(msgs.NumMessages)]);
end

% --- 设置绘图风格 (Journal Style) ---
set(0, 'DefaultAxesFontName', 'Microsoft YaHei'); % 修改此处：使用支持中文的字体，解决乱码问题
set(0, 'DefaultAxesFontSize', 12);
set(0, 'DefaultLineLineWidth', 1.5);
set(0, 'DefaultFigureColor', 'w');
% 定义配色方案 (Blue, Red, Green)
c_blue = [0 0.4470 0.7410];
c_red = [0.8500 0.3250 0.0980];
c_green = [0.4660 0.6740 0.1880];
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

figure('Name', '末端受力与力矩');
subplot(2,1,1);
plot(fext_time, Fx, 'Color', c_blue); hold on;
plot(fext_time, Fy, 'Color', c_red);
plot(fext_time, Fz, 'Color', c_green);
title('末端受力变化曲线', 'FontWeight', 'bold');
xlabel('时间 (s)');
ylabel('力 (N)');
legend('F_x','F_y','F_z', 'Location', 'best', 'Box', 'off');
grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(2,1,2);
plot(fext_time, Tx, 'Color', c_blue); hold on;
plot(fext_time, Ty, 'Color', c_red);
plot(fext_time, Tz, 'Color', c_green);
title('末端力矩变化曲线', 'FontWeight', 'bold');
xlabel('时间 (s)');
ylabel('力矩 (Nm)');
legend('T_x','T_y','T_z', 'Location', 'best', 'Box', 'off');
grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

% 2. 各关节力矩（/franka_state_controller/joint_states）
joint_msgs = select(bag, 'Topic', '/franka_state_controller/joint_states');
joint_structs = readMessages(joint_msgs, 'DataFormat', 'struct');
joint_time = joint_msgs.MessageList.Time - joint_msgs.MessageList.Time(1);
efforts = cell2mat(cellfun(@(m) m.Effort(:)', joint_structs, 'UniformOutput', false));

figure('Name', '各关节力矩');
plot(joint_time, efforts);
title('各关节力矩变化曲线', 'FontWeight', 'bold');
xlabel('时间 (s)');
ylabel('力矩 (Nm)');
legend('关节1','关节2','关节3','关节4','关节5','关节6','关节7', 'Location', 'bestoutside', 'Box', 'off');
grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

% 读取第二个 rosbag 文件
bag2 = rosbag('F:\Home\practice_data\franka_catch_without_pre-optimization_record_with_zmin_02_consider.bag');
% bag2 = rosbag('/home/q1ren/franka_catch_without_pre-configuration_record1.bag');

% 提取第二个文件的关节力矩
joint_msgs2 = select(bag2, 'Topic', '/franka_state_controller/joint_states');
joint_structs2 = readMessages(joint_msgs2, 'DataFormat', 'struct');
joint_time2 = joint_msgs2.MessageList.Time - joint_msgs2.MessageList.Time(1);
joint_time2 = joint_time2 - 0.622; % 时间轴前移3.911秒

efforts2 = cell2mat(cellfun(@(m) m.Effort(:)', joint_structs2, 'UniformOutput', false));
% 调试：检查两个数据源的幅值
disp(['bag1 (有预配置) 最大力矩: ', num2str(max(abs(efforts(:))))]);
disp(['bag2 (无预配置) 最大力矩: ', num2str(max(abs(efforts2(:))))]);
% 对比绘图（关节力矩）
num_joints = max(size(efforts,2), size(efforts2,2));
figure('Name', '关节力矩对比 (2组)', 'Position', [100, 100, 1000, 800]);
for j = 1:num_joints
    subplot(4,2,j);
    has1 = size(efforts,2) >= j;
    has2 = size(efforts2,2) >= j;
    plotted = false;
    if has1 && ~isempty(efforts(:,j))
        plot(joint_time, efforts(:,j), 'Color', c_blue, 'DisplayName', '有预配置');
        plotted = true;
        hold on;
    end
    if has2 && ~isempty(efforts2(:,j))
        plot(joint_time2, efforts2(:,j), 'Color', c_red, 'LineStyle', '--', 'DisplayName', '无预配置');
        plotted = true;
    end
    if plotted
        hold off;
        title(['关节', num2str(j), '力矩对比'], 'FontWeight', 'bold');
        xlabel('时间 (s)');
        ylabel('力矩 (Nm)');
        if j == 1, legend('Location', 'best', 'Box', 'off'); end
        grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;
    else
        title(['关节', num2str(j), '无数据']);
    end
end

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

% 对比末端受力
figure('Name', '末端受力/力矩对比 (2组)');
subplot(2,1,1);
plot(fext_time, Fx, 'Color', c_blue, 'DisplayName', 'F_x 有预配置'); hold on;
plot(fext_time2, Fx2, 'Color', c_blue, 'LineStyle', '--', 'DisplayName', 'F_x 无预配置');
plot(fext_time, Fy, 'Color', c_red, 'DisplayName', 'F_y 有预配置');
plot(fext_time2, Fy2, 'Color', c_red, 'LineStyle', '--', 'DisplayName', 'F_y 无预配置');
plot(fext_time, Fz, 'Color', c_green, 'DisplayName', 'F_z 有预配置');
plot(fext_time2, Fz2, 'Color', c_green, 'LineStyle', '--', 'DisplayName', 'F_z 无预配置');
hold off;
title('末端受力对比', 'FontWeight', 'bold');
xlabel('时间 (s)');
ylabel('力 (N)');
legend('NumColumns', 3, 'Location', 'best', 'Box', 'off');
grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

% 对比末端力矩
subplot(2,1,2);
plot(fext_time, Tx, 'Color', c_blue, 'DisplayName', 'T_x 有预配置'); hold on;
plot(fext_time2, Tx2, 'Color', c_blue, 'LineStyle', '--', 'DisplayName', 'T_x 无预配置');
plot(fext_time, Ty, 'Color', c_red, 'DisplayName', 'T_y 有预配置');
plot(fext_time2, Ty2, 'Color', c_red, 'LineStyle', '--', 'DisplayName', 'T_y 无预配置');
plot(fext_time, Tz, 'Color', c_green, 'DisplayName', 'T_z 有预配置');
plot(fext_time2, Tz2, 'Color', c_green, 'LineStyle', '--', 'DisplayName', 'T_z 无预配置');
hold off;
title('末端力矩对比', 'FontWeight', 'bold');
xlabel('时间 (s)');
ylabel('力矩 (Nm)');
legend('NumColumns', 3, 'Location', 'best', 'Box', 'off');
grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

% 计算总受力和总力矩
F_total = sqrt(Fx.^2 + Fy.^2 + Fz.^2);
F_total2 = sqrt(Fx2.^2 + Fy2.^2 + Fz2.^2);
T_total = sqrt(Tx.^2 + Ty.^2 + Tz.^2);
T_total2 = sqrt(Tx2.^2 + Ty2.^2 + Tz2.^2);

% 分量和总受力对比
figure('Name', '受力分量对比 (2组)', 'Position', [100, 100, 800, 800]);
subplot(4,1,1);
plot(fext_time, Fx, 'Color', c_blue); hold on; plot(fext_time2, Fx2, 'Color', c_red, 'LineStyle', '--');
title('F_x 分量对比', 'FontWeight', 'bold'); ylabel('F_x (N)'); legend('有预配置','无预配置', 'Box', 'off'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,2);
plot(fext_time, Fy, 'Color', c_blue); hold on; plot(fext_time2, Fy2, 'Color', c_red, 'LineStyle', '--');
title('F_y 分量对比', 'FontWeight', 'bold'); ylabel('F_y (N)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,3);
plot(fext_time, Fz, 'Color', c_blue); hold on; plot(fext_time2, Fz2, 'Color', c_red, 'LineStyle', '--');
title('F_z 分量对比', 'FontWeight', 'bold'); ylabel('F_z (N)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,4);
plot(fext_time, F_total, 'Color', c_blue); hold on; plot(fext_time2, F_total2, 'Color', c_red, 'LineStyle', '--');
title('总受力对比', 'FontWeight', 'bold'); xlabel('时间 (s)'); ylabel('总受力 (N)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

% 分量和总力矩对比
figure('Name', '力矩分量对比 (2组)', 'Position', [100, 100, 800, 800]);
subplot(4,1,1);
plot(fext_time, Tx, 'Color', c_blue); hold on; plot(fext_time2, Tx2, 'Color', c_red, 'LineStyle', '--');
title('T_x 分量对比', 'FontWeight', 'bold'); ylabel('T_x (Nm)'); legend('有预配置','无预配置', 'Box', 'off'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,2);
plot(fext_time, Ty, 'Color', c_blue); hold on; plot(fext_time2, Ty2, 'Color', c_red, 'LineStyle', '--');
title('T_y 分量对比', 'FontWeight', 'bold'); ylabel('T_y (Nm)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,3);
plot(fext_time, Tz, 'Color', c_blue); hold on; plot(fext_time2, Tz2, 'Color', c_red, 'LineStyle', '--');
title('T_z 分量对比', 'FontWeight', 'bold'); ylabel('T_z (Nm)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,4);
plot(fext_time, T_total, 'Color', c_blue); hold on; plot(fext_time2, T_total2, 'Color', c_red, 'LineStyle', '--');
title('总力矩对比', 'FontWeight', 'bold'); xlabel('时间 (s)'); ylabel('总力矩 (Nm)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

% 读取第三个 rosbag 文件
bag3 = rosbag('F:\Home\practice_data\franka_catch_with_only_TO_record1.bag');
delta_to = 18.496 - 8.628; % 第三个文件比第一个文件早的时间差

% 提取第三个文件的关节力矩
joint_msgs3 = select(bag3, 'Topic', '/franka_state_controller/joint_states');
joint_structs3 = readMessages(joint_msgs3, 'DataFormat', 'struct');
joint_time3 = joint_msgs3.MessageList.Time - joint_msgs3.MessageList.Time(1);
joint_time3 = joint_time3 + delta_to; % 时间轴后移对齐
efforts3 = cell2mat(cellfun(@(m) m.Effort(:)', joint_structs3, 'UniformOutput', false));

disp(['bag3 (仅TO) 最大力矩: ', num2str(max(abs(efforts3(:))))]);

% 对比绘图（关节力矩 - 三个文件）
num_joints = max([size(efforts,2), size(efforts2,2), size(efforts3,2)]);
figure('Name', '关节力矩对比 (3组)', 'Position', [100, 100, 1000, 800]);
for j = 1:num_joints
    subplot(4,2,j);
    has1 = size(efforts,2) >= j;
    has2 = size(efforts2,2) >= j;
    has3 = size(efforts3,2) >= j;
    plotted = false;
    if has1 && ~isempty(efforts(:,j))
        plot(joint_time, efforts(:,j), 'Color', c_blue, 'DisplayName', '有预配置');
        plotted = true;
        hold on;
    end
    if has2 && ~isempty(efforts2(:,j))
        plot(joint_time2, efforts2(:,j), 'Color', c_red, 'LineStyle', '--', 'DisplayName', '无预配置');
        plotted = true;
    end
    if has3 && ~isempty(efforts3(:,j))
        plot(joint_time3, efforts3(:,j), 'Color', c_green, 'LineStyle', ':', 'DisplayName', '仅TO');
        plotted = true;
    end
    if plotted
        hold off;
        title(['关节', num2str(j), '力矩对比'], 'FontWeight', 'bold');
        xlabel('时间 (s)');
        ylabel('力矩 (Nm)');
        if j == 1, legend('Location', 'best', 'Box', 'off'); end
        grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;
    else
        title(['关节', num2str(j), '无数据']);
    end
end

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

% 对比末端受力（三个文件）
figure('Name', '末端受力对比 (3组)');
subplot(2,1,1);
plot(fext_time, Fx, 'Color', c_blue, 'DisplayName', 'F_x 有预配置'); hold on;
plot(fext_time2, Fx2, 'Color', c_blue, 'LineStyle', '--', 'DisplayName', 'F_x 无预配置');
plot(fext_time3, Fx3, 'Color', c_blue, 'LineStyle', ':', 'DisplayName', 'F_x 仅TO');

plot(fext_time, Fy, 'Color', c_red, 'DisplayName', 'F_y 有预配置');
plot(fext_time2, Fy2, 'Color', c_red, 'LineStyle', '--', 'DisplayName', 'F_y 无预配置');
plot(fext_time3, Fy3, 'Color', c_red, 'LineStyle', ':', 'DisplayName', 'F_y 仅TO');

plot(fext_time, Fz, 'Color', c_green, 'DisplayName', 'F_z 有预配置');
plot(fext_time2, Fz2, 'Color', c_green, 'LineStyle', '--', 'DisplayName', 'F_z 无预配置');
plot(fext_time3, Fz3, 'Color', c_green, 'LineStyle', ':', 'DisplayName', 'F_z 仅TO');
hold off;
title('末端受力对比', 'FontWeight', 'bold');
xlabel('时间 (s)');
ylabel('力 (N)');
legend('NumColumns', 3, 'Location', 'best', 'Box', 'off');
grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

% 对比末端力矩（三个文件）
subplot(2,1,2);
plot(fext_time, Tx, 'Color', c_blue, 'DisplayName', 'T_x 有预配置'); hold on;
plot(fext_time2, Tx2, 'Color', c_blue, 'LineStyle', '--', 'DisplayName', 'T_x 无预配置');
plot(fext_time3, Tx3, 'Color', c_blue, 'LineStyle', ':', 'DisplayName', 'T_x 仅TO');

plot(fext_time, Ty, 'Color', c_red, 'DisplayName', 'T_y 有预配置');
plot(fext_time2, Ty2, 'Color', c_red, 'LineStyle', '--', 'DisplayName', 'T_y 无预配置');
plot(fext_time3, Ty3, 'Color', c_red, 'LineStyle', ':', 'DisplayName', 'T_y 仅TO');

plot(fext_time, Tz, 'Color', c_green, 'DisplayName', 'T_z 有预配置');
plot(fext_time2, Tz2, 'Color', c_green, 'LineStyle', '--', 'DisplayName', 'T_z 无预配置');
plot(fext_time3, Tz3, 'Color', c_green, 'LineStyle', ':', 'DisplayName', 'T_z 仅TO');
hold off;
title('末端力矩对比', 'FontWeight', 'bold');
xlabel('时间 (s)');
ylabel('力矩 (Nm)');
legend('NumColumns', 3, 'Location', 'best', 'Box', 'off');
grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

% 计算第三个文件的总受力和总力矩
F_total3 = sqrt(Fx3.^2 + Fy3.^2 + Fz3.^2);
T_total3 = sqrt(Tx3.^2 + Ty3.^2 + Tz3.^2);

% 分量和总受力对比（三个文件）
figure('Name', '受力分量对比 (3组)', 'Position', [100, 100, 800, 800]);
subplot(4,1,1);
plot(fext_time, Fx, 'Color', c_blue); hold on;
plot(fext_time2, Fx2, 'Color', c_red, 'LineStyle', '--');
plot(fext_time3, Fx3, 'Color', c_green, 'LineStyle', ':'); hold off;
title('F_x 分量对比', 'FontWeight', 'bold'); ylabel('F_x (N)'); legend('有预配置','无预配置','仅TO', 'Box', 'off'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,2);
plot(fext_time, Fy, 'Color', c_blue); hold on;
plot(fext_time2, Fy2, 'Color', c_red, 'LineStyle', '--');
plot(fext_time3, Fy3, 'Color', c_green, 'LineStyle', ':'); hold off;
title('F_y 分量对比', 'FontWeight', 'bold'); ylabel('F_y (N)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,3);
plot(fext_time, Fz, 'Color', c_blue); hold on;
plot(fext_time2, Fz2, 'Color', c_red, 'LineStyle', '--');
plot(fext_time3, Fz3, 'Color', c_green, 'LineStyle', ':'); hold off;
title('F_z 分量对比', 'FontWeight', 'bold'); ylabel('F_z (N)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,4);
plot(fext_time, F_total, 'Color', c_blue); hold on;
plot(fext_time2, F_total2, 'Color', c_red, 'LineStyle', '--');
plot(fext_time3, F_total3, 'Color', c_green, 'LineStyle', ':'); hold off;
title('总受力对比', 'FontWeight', 'bold'); xlabel('时间 (s)'); ylabel('总受力 (N)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

% 分量和总力矩对比（三个文件）
figure('Name', '力矩分量对比 (3组)', 'Position', [100, 100, 800, 800]);
subplot(4,1,1);
plot(fext_time, Tx, 'Color', c_blue); hold on;
plot(fext_time2, Tx2, 'Color', c_red, 'LineStyle', '--');
plot(fext_time3, Tx3, 'Color', c_green, 'LineStyle', ':'); hold off;
title('T_x 分量对比', 'FontWeight', 'bold'); ylabel('T_x (Nm)'); legend('有预配置','无预配置','仅TO', 'Box', 'off'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,2);
plot(fext_time, Ty, 'Color', c_blue); hold on;
plot(fext_time2, Ty2, 'Color', c_red, 'LineStyle', '--');
plot(fext_time3, Ty3, 'Color', c_green, 'LineStyle', ':'); hold off;
title('T_y 分量对比', 'FontWeight', 'bold'); ylabel('T_y (Nm)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,3);
plot(fext_time, Tz, 'Color', c_blue); hold on;
plot(fext_time2, Tz2, 'Color', c_red, 'LineStyle', '--');
plot(fext_time3, Tz3, 'Color', c_green, 'LineStyle', ':'); hold off;
title('T_z 分量对比', 'FontWeight', 'bold'); ylabel('T_z (Nm)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

subplot(4,1,4);
plot(fext_time, T_total, 'Color', c_blue); hold on;
plot(fext_time2, T_total2, 'Color', c_red, 'LineStyle', '--');
plot(fext_time3, T_total3, 'Color', c_green, 'LineStyle', ':'); hold off;
title('总力矩对比', 'FontWeight', 'bold'); xlabel('时间 (s)'); ylabel('总力矩 (Nm)'); grid on; set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2); box on;

% % 末端轨迹（从 /gazebo/link_states 获取）
% link_states_msgs = select(bag, 'Topic', '/gazebo/link_states');
% link_states_structs = readMessages(link_states_msgs, 'DataFormat', 'struct');
% link_time = link_states_msgs.MessageList.Time - link_states_msgs.MessageList.Time(1);

% % 假设末端 link 名为 'franka::panda_plate'
% link_names = link_states_structs{1}.Name;
% ee_idx = find(strcmp(link_names, 'panda::panda_plate'), 1);
% if isempty(ee_idx)
%     error('未找到末端 link "panda::panda_plate"');
% end
% % plate质心位置
% ee_xyz = cell2mat(cellfun(@(m) [m.Pose(ee_idx).Position.X, m.Pose(ee_idx).Position.Y, m.Pose(ee_idx).Position.Z], link_states_structs, 'UniformOutput', false));
% % plate上表面
% ee_xyz(:,3) = ee_xyz(:,3) + 0.01/2;

% % 物体轨迹（名称为 stone）
% model_states_msgs = select(bag, 'Topic', '/gazebo/model_states');
% model_states_structs = readMessages(model_states_msgs, 'DataFormat', 'struct');
% model_names = model_states_structs{1}.Name;
% obj_idx = find(strcmp(model_names, 'stone'), 1);
% if isempty(obj_idx)
%     error('未找到物体 "stone"');
% end
% % stone质心位置
% obj_xyz = cell2mat(cellfun(@(m) [m.Pose(obj_idx).Position.X, m.Pose(obj_idx).Position.Y, m.Pose(obj_idx).Position.Z], model_states_structs, 'UniformOutput', false));
% % stone下表面
% obj_xyz(:,3) = obj_xyz(:,3) - 0.050/2;

% % 绘制三维轨迹
% figure;
% plot3(ee_xyz(:,1), ee_xyz(:,2), ee_xyz(:,3), 'b', 'LineWidth', 2);
% hold on;
% plot3(obj_xyz(:,1), obj_xyz(:,2), obj_xyz(:,3), 'r', 'LineWidth', 2);
% hold off;
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z (m)');
% title('末端上表面与物体下表面三维轨迹');
% legend('plate上表面轨迹','stone下表面轨迹');
% grid on;