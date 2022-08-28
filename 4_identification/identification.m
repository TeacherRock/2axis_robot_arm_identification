clc ; clear ; close all ;

Axis = 2;
sampTs = 0.001;  % 取樣時間

for i = 1 : 5
    MeasuredData = load("D:/成大/碩一/新訓/我的/6_二軸手臂鑑別/4_identification/data/output/data" + int2str(i) + ".txt");
    PosCmd(:, :, i) = MeasuredData(:, 1 + Axis*0 : Axis + Axis*0);
    VelCmd(:, :, i) = MeasuredData(:, 1 + Axis*1 : Axis + Axis*1);
    AccCmd(:, :, i) = MeasuredData(:, 1 + Axis*2 : Axis + Axis*2);
    Pos(:, :, i)    = MeasuredData(:, 1 + Axis*3 : Axis + Axis*3);
    Tor(:, :, i) = MeasuredData(:, 1 + Axis*5 : Axis + Axis*5);
    Data(i) = dataprocess(Pos(:, :, i), Tor(:, :, i));
end

%% 鑑別參數

% 比較誤差 採用扭矩誤差
% [Theta, Analyze] = id(Data1.fitlter_avgPos, Data1.fitlter_avgVel, Data1.fitlter_avgAcc, Data1.avgTor);
% Final1 = [Theta.theta_LS, Theta.theta_WLS, Analyze.RSD_LS, Analyze.RSD_WLS];
% T = array2table(Final1, 'VariableNames', {'theta_LS', 'theta_WLS', 'RSD_LS', 'RSD_WLS'});
% T.Properties.RowNames = {'m2*d1^2+Ia1+Lzz1'; 'Lzz2'; 'Ix2'; 'Iy2'; 'Ia2'; 'fs1'; 'fs2'; 'fv1'; 'fv2'};  

for i = 1 : 5
    for j = 1 : 5
        record = ploterror(Data(i), Data(j), i, j);
        error1(i, j) = record.error1;
        error2(i, j) = record.error2;
    end
    theta.LS(:, i) = record.Theta_LS;
    thets.WLS(:, i) = record.Theta_WLS;

end

%% 平均百分物差

avg_e(1, :) = mean(error1, 1);
avg_e(2, :) = mean(error2, 1);

%% 儲存參數
fid = fopen('theta.txt','wt');
for i = 1 : 5
    fprintf(fid, 'Track%.0f\n', i);
    fprintf(fid, 'LS : \n');
    fprintf(fid, '%.4f\n', theta.LS(:, i));
    fprintf(fid, '\n');
    fprintf(fid, 'WLS : \n');
    fprintf(fid, '%.4f\n', thets.WLS(:, i));
    fprintf(fid, '\n');

end
fclose(fid);
%% 儲存誤差
fid = fopen('error.txt','wt');
for i = 1 : 5
    fprintf(fid, "### ID_"+ int2str(i) + " ###\n");
    for j = 1 : 5
        fprintf(fid,"Track : " + int2str(j) + "\n");
        fprintf(fid, 'Axis1 : %.2f %% \n', error1(i, j));
        fprintf(fid, 'Axis2 : %.2f %% \n', error2(i, j));
        fprintf(fid,' \n');
        fprintf(fid, '\n');
    end
end
fclose(fid);


% e = [error.error, error.error2];
% T = array2table(e, 'VariableNames', {'axis1', 'axis2'});
% 
% f = uifigure;
% t = uitable(f,'Data',T);

%% plot  input1:鑑別參數用  input2:驗證用
function output = ploterror(Data1, Data2, i, j)
sampTs = 0.001;  % 取樣時間
[Theta, Analyze] = id(Data1.fitlter_avgPos, Data1.fitlter_avgVel, Data1.fitlter_avgAcc, Data1.avgTor);

tor_id = Wmatrix(Data2.fitlter_avgPos, Data2.fitlter_avgVel, Data2.fitlter_avgAcc)*Theta.theta_WLS;
t1 = sampTs : sampTs : length(tor_id)*sampTs/2;
figure(1)
plot(t1, Data2.avgTor(1:end/2), t1, tor_id(1:end/2))
title("Axis1");
legend("力矩(實際)", "力矩(鑑別)")
xlabel("time (s)"); ylabel("torque (N*m)");
saveas(gcf, "pic/Axis1_Tor_" + int2str(i) + '_' + int2str(j) + ".jpg")

figure(2)
plot(t1, Data2.avgTor(end/2+1:end), t1, tor_id(end/2+1:end))
title("Axis2");
legend("力矩(實際)", "力矩(鑑別)")
xlabel("time (s)"); ylabel("torque (N*m)");
saveas(gcf, "pic/Axis2_Tor_" + int2str(i) + '_' + int2str(j) + ".jpg")

output.error1 = norm(Data2.avgTor(1:end/2) - tor_id(1:end/2)) / norm(Data2.avgTor(1:end/2)) * 100;
output.error2 = norm(Data2.avgTor(end/2+1:end) - tor_id(end/2+1:end)) / norm(Data2.avgTor(end/2+1:end)) * 100;
output.Theta_LS = Theta.theta_LS;
output.Theta_WLS = Theta.theta_WLS;

end

%% 鑑別
function [Theta, Analyze] = id(Pos, Vel, Acc, Tor)

% 回歸矩陣
W = Wmatrix(Pos, Vel, Acc);
n = length(Pos(:, 1));

% 平行抽取濾波 : 降低取樣頻率(1/10)
Tor = decimate(Tor, 10);
tempW = zeros(2*n/10, 9); 
for i = 1 : 9
    tempW(:, i) = decimate(W(:, i), 10);  
end
W = tempW;

n_filter = length(Tor/2);

% 最小平方法(sudo inverse)
Theta.theta_LS = pinv(W)*Tor; 
Tor_LS = W*Theta.theta_LS;

   % 相對標準偏差 Relative Standard Deviation
Analyze.Sigma_Error_LS = sqrt( (norm(Tor - Tor_LS)^2 )/( 2 * n_filter - 9 ) ) ; % 力矩誤差
Analyze.Sigma_theta_LS = sqrt( diag( Analyze.Sigma_Error_LS^2*inv(W'*W) ) ) ; % ??
Analyze.RSD_LS = ( Analyze.Sigma_theta_LS ./ abs( Theta.theta_LS ) ) * 100 ; % 標準差

% 加權最小平方法 Weighted Least Squares
for i = 0 : 1
    Sigma_JointError_LS = sqrt( (norm(Tor((1200*i + 1):1200*(i + 1), 1) - Tor_LS((1200*i + 1):1200*(i + 1), 1))^2)/( n_filter/2 - 9 ) );
    Tor_WLS((1200*i + 1):1200*(i + 1), 1) = Tor((1200*i + 1):1200*(i + 1))/Sigma_JointError_LS;
    W_WLS((1200*i + 1):1200*(i + 1), :) = W((1200*i + 1):1200*(i + 1), :)/Sigma_JointError_LS;
end

Theta.theta_WLS = (W_WLS)\Tor_WLS;
   % 相對標準偏差 Relative Standard Deviation
Analyze.Sigma_theta_WLS = sqrt( diag(inv(W_WLS'*W_WLS) ) ) ; % ??
Analyze.RSD_WLS = ( Analyze.Sigma_theta_WLS./abs( Theta.theta_WLS) ) * 100 ; % 標準差

Analyze.ConditionNum = cond(W);


end
%% 
function Output = Wmatrix(Pos, Vel, Acc)
n = length(Pos(:, 1));
W = zeros(2*n, 9);
d1 = 0.24;
for i = 1 : n
    W(i, :) = [ (Acc(i, 1)), (Acc(i, 1) + Acc(i, 2)), (-d1*(sin(Pos(i, 2))*Vel(i, 2)^2 + 2*Vel(i, 1)*sin(Pos(i, 2))*Vel(i, 2) - 2*Acc(i, 1)*cos(Pos(i, 2)) - Acc(i, 2)*cos(Pos(i, 2)))), (-d1*(cos(Pos(i, 2))*Vel(i, 2)^2 + 2*Vel(i, 1)*cos(Pos(i, 2))*Vel(i, 2) + 2*Acc(i, 1)*sin(Pos(i, 2)) + Acc(i, 2)*sin(Pos(i, 2)))), 0,  sign(Vel(i, 1)), 0, Vel(i, 1), 0];
    W(i + n, :) = [ 0, (Acc(i, 1) + Acc(i, 2)), (d1*(sin(Pos(i, 2))*Vel(i, 1)^2 + Acc(i, 1)*cos(Pos(i, 2)))), (d1*(cos(Pos(i, 2))*Vel(i, 1)^2 - Acc(i, 1)*sin(Pos(i, 2)))), Acc(i, 2), 0, sign(Vel(i, 2)), 0,  Vel(i, 2)];
end
Output = W;
end

%% 資料處理
function Output = dataprocess(Pos, Tor)
% 參數
Axis = 2;
sampTs = 0.001;  % 取樣時間
tf = length(Pos(:, 1))*sampTs/10;  % 終止時間
fm = 1 / sampTs ;  % 取樣頻率 (Hz)
nh = 5 ;  % 諧波數
wf = ( 2 * pi ) / tf ;  % 基礎頻率 (rad/s) , 單位轉換: (rad/s) = ( 2 * pi ) * (Hz)
fdyn = nh * ( wf / ( 2 * pi ) ) ;  % 激發軌跡最大頻率 = 系統動態頻率 (Hz)
Order = 3 ;  % 階數(因速度與加速度必須大於2)
CutoffFreq = 10 * fdyn ;  % 截止頻率 (Hz)

% 多週期(10週期) 取平均
[avgPos, avgTor] = deal(zeros(tf/sampTs, Axis));
for i = 1 : 10
    avgPos = avgPos + Pos(12000*(i-1) + 1: 12000*i, :);
    avgTor = avgTor + Tor(12000*(i-1) + 1: 12000*i, :);
end
avgPos = avgPos/10;
avgTor = avgTor/10;
avgTor = [avgTor(:, 1); avgTor(:, 2)];


% 濾波(對位置濾波，再微分取得速度、加速度)
[ b , a ] = butter( Order , CutoffFreq / ( fm / 2 ) , 'low' ) ;  % Butterworth 低通濾波器 , 正規化 : CutoffFreq / ( Sampling / 2 )
fitlter_avgPos = filtfilt( b , a , avgPos ) ;  % 零相位數位濾波器 : 正方向與反方向時間各濾波一次，可得出零相位失真
fitlter_avgVel = derivate(fitlter_avgPos, 1, sampTs, Axis);
fitlter_avgAcc = derivate(fitlter_avgPos, 2, sampTs, Axis);

Output.fitlter_avgPos = fitlter_avgPos;
Output.fitlter_avgVel = fitlter_avgVel;
Output.fitlter_avgAcc = fitlter_avgAcc;
Output.avgTor = avgTor;

end