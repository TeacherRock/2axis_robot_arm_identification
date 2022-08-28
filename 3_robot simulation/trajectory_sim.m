%% Load Command
openfile = 'Trajectory4_10times.txt';
path  = ['D:\成大\碩一\新訓\我的\6_二軸手臂鑑別\3_robot simulation\Trajectory\', openfile];
Trajectory = load(path);
trajectory.P = Trajectory(:, 1 : 2);
trajectory.V = Trajectory(:, 3 : 4);
trajectory.A = Trajectory(:, 5 : 6);
sampTs = 0.001;
tf = 12;
RatedSpeed = 3000 * 2*pi/60 ;  % 馬達額定轉速 (rad/s) , 單位轉換: (rad/s) = (rpm) * 2*pi/60
GearRatio = [ 50 , 50 ] ;  % 各軸減速比
PosBound = [  90  ,  150  ;
             -90  , -150  ] * 0.7 * ( pi / 180 ) ;  % 機構角度限制 (rad)     
VelBound = ( RatedSpeed ./ GearRatio ) * 0.8 ;  % 機構速度限制 (rad/s)
AccBound = VelBound * 2 ;  % 機構加速度限制: 速度限制*自定倍數 (rad/s^2) 

%% 位置 速度 加速度
figure()
t = sampTs : sampTs : tf;
for i = 1 : 2
    if i == 1
        ax = " (Axis1)";
    else
        ax = " (Axis2)";
    end
    subplot(3, 2, i)
    plot(t, trajectory.P(1:end/10, i));
    title("Position" + ax)
    yline(fliplr(PosBound(:, i)'), 'r-')
    xlabel("time (s)"); ylabel("rad")
    ylim([1.4*min(trajectory.P(:, i)), 1.4*max(trajectory.P(:, i))])
    
    subplot(3, 2, i + 2)
    plot(t, trajectory.V(1:end/10, i))
    title("Velocity" + ax)
    yline([-1, 1] * VelBound(i), 'r-')
    xlabel("time (s)"); ylabel("rad/s")
    ylim([-1.4*VelBound(i), 1.4*VelBound(i)])
    
    subplot(3, 2, i + 4)
    plot(t, trajectory.A(1:end/10, i))
    title("Acceleration" + ax)
    yline([-1, 1] * AccBound(i), 'r-')
    xlabel("time (s)"); ylabel("rad/s^2")
    ylim([-1.4*AccBound(i), 1.4*AccBound(i)])
end
%% 順項運動學
% l1 = 0.24; l2 = 0.24;
% % 末端位置
% x = l1*cos(trajectory.P(:, 1)) + l2*cos(trajectory.P(:, 1) + trajectory.P(:, 2));
% y = l1*sin(trajectory.P(:, 1)) + l2*sin(trajectory.P(:, 1) + trajectory.P(:, 2));
% 
% % 關節1位置
% x1 = l1*cos(trajectory.P(:, 1));
% y1 = l1*sin(trajectory.P(:, 1));
% pic_num = 1;
% pic_num = 1;
% for i = 1 : 100 : length(x)/10 + 1
%     figure(1)
%     plot([0, x1(i),x(i)], [0, y1(i),y(i)],'-o', x(1:i), y(1:i))
%     title("t = " + j + " s")
%     j = j + 0.1;
%     xlim([-0.5 0.5]); ylim([-0.5 0.5])
%     xlabel("x (m)"); ylabel("y (m)");
%     pause(0.1); 
%     drawnow;
%     F = getframe(gcf);
%     I = frame2im(F);
%     [I,map]=rgb2ind(I,256);
%     
%     if pic_num == 1
%         imwrite(I,map,'test2.gif','gif','Loopcount',inf,'DelayTime',0.2);
%     else
%         imwrite(I,map,'test2.gif','gif','WriteMode','append','DelayTime',0.2);
%     end
%     
%     pic_num = pic_num + 1;
% end
% 
% %% 末端軌跡圖
% figure(2)
% plot(x,y)