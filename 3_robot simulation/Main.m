clc; close all; clear;
cd('D:\成大\碩一\新訓\我的\6_二軸手臂鑑別\3_robot simulation')
addpath('robot', 'Utils\')

%% Load Command
openfile = 'Trajectory1_10times.txt';
path  = ['D:\成大\碩一\新訓\我的\6_二軸手臂鑑別\3_robot simulation\Trajectory\', openfile];
Trajectory = load(path);
cmd.pos = Trajectory(:, 1 : 2);
cmd.vel = Trajectory(:, 3 : 4);
cmd.acc = Trajectory(:, 5 : 6);
cmd.length = size(cmd.pos, 1);
cmd.samplingtime = 0.001;

t = 1 : length(cmd.pos(:, 1))/10;
plot(t, cmd.pos(t, :))
legend('axis1', 'axis2')
title('position')


%% Create Robot Handle
ts = 0.001; 
rb = Robot(ts, 'SCARA');

%% Controller Settings
cr = controller();

%% Robot Tracking Control
record = rb.Tracking(cmd, cr);

%% Plot Data =========================================================
PlotData(record)

%% Collect Data
% Data = [record.q', record.q_dot', record.q_ddot', record.torque'];
% savefile = 'record.txt';
% savepath = ['D:\成大\碩一\新訓\我的\6_二軸手臂鑑別\3_robot simulation\record_data\', savefile];
% Generate_txt(Data, savepath);


%% Function ==========================================================
function PlotData(record)

figure()
for i = 1 : 2
    subplot(2, 1, i)
    plot(record.t, record.q(i, :), record.t, record.cmd.pos(:, i), '--');
    ylabel('Pos[rad]')
    xlabel('Time[sec]')
    title(['Joint', num2str(i)])
    legend('simulation result','command')
    grid on
end

figure()
for i = 1 : 2
    subplot(2, 1, i)
    plot(record.t, record.q_dot(i, :), record.t, record.cmd.vel(:, i), '--');
    ylabel('Vel[rad/s]')
    xlabel('Time[sec]')
    title(['Joint', num2str(i)])
    legend('simulation result','command')
    grid on
end

figure()
for i = 1 : 2
    subplot(2, 1, i)
    plot(record.t, record.q_ddot(i, :), record.t, record.cmd.acc(:, i), '--');
    ylabel('Acc[rad/s^2]')
    xlabel('Time[sec]')
    title(['Joint', num2str(i)])
    legend('simulation result','command')
    grid on
end

figure()
for i = 1 : 2
    subplot(2, 1, i)
    plot(record.t, record.torque(i, :));
    ylabel('Torque[Nm]')
    
    xlabel('Time[sec]')
    title(['Joint', num2str(i)])
    grid on
end


end
