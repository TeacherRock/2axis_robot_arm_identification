clc; close all; clear;
%% 空間限制   
% A*x < b
sampTi = 0.02;
t0 = 0;
tf = 12;
wf = (2 * pi)/tf ;
N = 5;
n = tf/sampTi;
A = zeros(6*n, 2*(2*N+1));
b = zeros(6*n, 1);
i = 1 : 5;
k = 1 : 12;

RatedSpeed = 3000 * 2*pi/60 ;  % 馬達額定轉速 (rad/s) , 單位轉換: (rad/s) = (rpm) * 2*pi/60
GearRatio = [ 50 , 50 ] ;  % 各軸減速比

PosBound = [  90  ,  150  ;
             -90  , -150  ] * 0.7 * ( pi / 180 ) ;  % 機構角度限制 (rad)     
VelBound = ( RatedSpeed ./ GearRatio ) * 0.8 ;  % 機構速度限制 (rad/s)
AccBound = VelBound * 2 ;  % 機構加速度限制: 速度限制*自定倍數 (rad/s^2)         

qb = [ PosBound' , VelBound' , AccBound' ] ;  % 激勵軌跡限制 (角度正、角度負、速度、加速度)

% A = [a11, a12, ... a15, b11, b12, ... b15, q1, a21, a22, ... a25, b21, b22, ... b25, q2 ]
% x = [s11, s12, ... s15, c11, c12, ... c15,  1, s21, s22, ... s25, c21, c22, ... c25,  1 ]
% sxy : s = sin, x = axis, y = 基底數 
for t = sampTi : sampTi : tf
    A(k, :) = [ % axis1
                sin(wf*i*t)./(wf*i), -cos(wf*i*t)./(wf*i),  1,                    zeros(1, 11);    % < , q
               -sin(wf*i*t)./(wf*i),  cos(wf*i*t)./(wf*i), -1,                    zeros(1, 11);    % > , q
                cos(wf*i*t),          sin(wf*i*t),          0,                    zeros(1, 11);    % < , dq
               -cos(wf*i*t),         -sin(wf*i*t),          0,                    zeros(1, 11);    % > , dq
               -sin(wf*i*t).*(wf*i),  cos(wf*i*t).*(wf*i),  0,                    zeros(1, 11);    % < , ddq
                sin(wf*i*t).*(wf*i), -cos(wf*i*t).*(wf*i),  0,                    zeros(1, 11);   % > , ddq
                % axis2
                zeros(1, 11),         sin(wf*i*t)./(wf*i), -cos(wf*i*t)./(wf*i),  1;    % < , q
                zeros(1, 11),        -sin(wf*i*t)./(wf*i),  cos(wf*i*t)./(wf*i), -1;    % > , q
                zeros(1, 11),         cos(wf*i*t),          sin(wf*i*t),          0;    % < , dq
                zeros(1, 11),        -cos(wf*i*t),         -sin(wf*i*t),          0;    % > , dq
                zeros(1, 11),        -sin(wf*i*t).*(wf*i),  cos(wf*i*t).*(wf*i),  0;   % < , ddq
                zeros(1, 11),         sin(wf*i*t).*(wf*i), -cos(wf*i*t).*(wf*i),  0    % > , ddq
              ];  
          
    b(k, :) = [qb(1, 1); -qb(1, 2); qb(1, 3); qb(1, 3); qb(1, 4); qb(1, 4); qb(2, 1); -qb(2, 2); qb(2, 3); qb(2, 3); qb(2, 4); qb(2, 4)]; 
    k = k + 12;  
    
end

% Aeq*x = beq ,在開始結束時速度加速度均為0, 位置起點終點一樣
Aeq  = [ % axis1
         - sin(wf*i*tf)./(wf*i), (-1 + cos(wf*i*tf))./(wf*i),  0,                    zeros(1, 11);    % = , q
          cos(wf*i*t0),          sin(wf*i*t0),          0,                    zeros(1, 11);    % = , dq
          cos(wf*i*tf),          sin(wf*i*tf),          0,                    zeros(1, 11);    % = , dq
         -sin(wf*i*t0).*(wf*i),  cos(wf*i*t0).*(wf*i),  0,                    zeros(1, 11);    % = , ddq
         -sin(wf*i*tf).*(wf*i),  cos(wf*i*tf).*(wf*i),  0,                    zeros(1, 11);    % = , ddq
        
         % axis2
         zeros(1, 11),         - sin(wf*i*tf)./(wf*i), (-1 + cos(wf*i*tf))./(wf*i),  0;    % = , q
         zeros(1, 11),         cos(wf*i*t0),          sin(wf*i*t0),          0;    % = , dq
         zeros(1, 11),         cos(wf*i*tf),          sin(wf*i*tf),          0;    % = , dq
         zeros(1, 11),        -sin(wf*i*t0).*(wf*i),  cos(wf*i*t0).*(wf*i),  0;    % = , ddq
         zeros(1, 11),        -sin(wf*i*tf).*(wf*i),  cos(wf*i*tf).*(wf*i),  0;    % = , ddq
         ];  
beq = zeros(10, 1);

%% 最佳化
xb = 1 ;  % x0範圍
x0 = 2 * rand( size( A , 2 ) , 1 ) * xb - xb ;  % x 初值

options = optimoptions( 'fmincon' , 'Algorithm' , 'sqp' , 'Display' ,'iter','PlotFcn',{@optimplotfval},'MaxIterations' ,2000 );
[ x , Index ] = fmincon( @(x)optfun(x) , x0 , A , b , Aeq , beq , [] , [] , [] , options ) ;  % 軌跡優化計算

sampTs = 0.001;
trajectory = genTrajectory(x, sampTs);

%% 命令圖

figure()
t = sampTs : sampTs : tf;
for i = 1 : 2
    if i == 1
        ax = "Axis1";
    else
        ax = "Axis2";
    end
    subplot(3, 2, i)
    plot(t, trajectory.P(:, i));
    title(ax + " Position")
    yline(fliplr(PosBound(:, i)'), 'r-')
    ylim([1.1*min(trajectory.P(:, i)), 1.1*max(trajectory.P(:, i))])
    
    subplot(3, 2, i + 2)
    plot(t, trajectory.V(:, i))
    title(ax + " Velocity")
    yline([-1, 1] * VelBound(i), 'r-')
    ylim([-1.1*VelBound(i), 1.1*VelBound(i)])
    
    subplot(3, 2, i + 4)
    plot(t, trajectory.A(:, i))
    title(ax + " Acceleration")
    yline([-1, 1] * AccBound(i), 'r-')
    ylim([-1.1*AccBound(i), 1.1*AccBound(i)])
end

%% 儲存 Trajectory
openfile = 'Trajectory5_10times.txt';
path  = ['D:\成大\碩一\新訓\我的\6_二軸手臂鑑別\3_robot simulation\Trajectory\', openfile];
fid = fopen(path,'wt');
i=1;

for t = 0 : sampTs : 10*tf-sampTs
    fprintf(fid,'%f\t',trajectory.P(i, 1));
    fprintf(fid,'%f\t',trajectory.P(i, 2));
    fprintf(fid,'%f\t',trajectory.V(i, 1));
    fprintf(fid,'%f\t',trajectory.V(i, 2));
    fprintf(fid,'%f\t',trajectory.A(i, 1));
    fprintf(fid,'%f\n',trajectory.A(i, 2));
    i = i+1;
    if(i == tf/sampTs)
        i = 1;
    elseif(i == 10*tf/sampTs)
        break;
    end
end
fclose(fid);

 
%% Load Command
% openfile = 'Trajectory5_10times.txt';
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

%% 順項運動學
l1 = 0.24; l2 = 0.24;
% 末端位置
x = l1*cos(trajectory.P(:, 1)) + l2*cos(trajectory.P(:, 1) + trajectory.P(:, 2));
y = l1*sin(trajectory.P(:, 1)) + l2*sin(trajectory.P(:, 1) + trajectory.P(:, 2));

% 關節1位置
x1 = l1*cos(trajectory.P(:, 1));
y1 = l1*sin(trajectory.P(:, 1));
pic_num = 1;
j = 0;
for i = 1 : 100 : length(x)/10 + 1
    figure(1)
    plot([0, x1(i),x(i)], [0, y1(i),y(i)],'-o', x(1:i), y(1:i))
    title("t = " + j + " s")
    j = j + 0.1;
    xlim([-0.5 0.5]); ylim([-0.5 0.5])
    xlabel("x (m)"); ylabel("y (m)");
    pause(0.1); 
    drawnow;
    F = getframe(gcf);
    I = frame2im(F);
    [I,map]=rgb2ind(I,256);
    
    if pic_num == 1
        imwrite(I,map,'test2.gif','gif','Loopcount',inf,'DelayTime',0.2);
    else
        imwrite(I,map,'test2.gif','gif','WriteMode','append','DelayTime',0.2);
    end
    
    pic_num = pic_num + 1;
end

%% 末端軌跡圖
% figure(3)
% plot(x,y)


