clc ; clear ; close all ;
MeasuredData = load('record.txt');
Axis = 2;
sampTs = 0.001;
tf = 12;
n = tf/sampTs - 1;
d1 = 0.24; d2 = 0.24;

Pos     = MeasuredData(:, 1 + Axis*0 : Axis + Axis*0);
Vel     = MeasuredData(:, 1 + Axis*1 : Axis + Axis*1);
Acc     = MeasuredData(:, 1 + Axis*2 : Axis + Axis*2);
Tor = MeasuredData(:, 1 + Axis*3 : Axis + Axis*3);
torque = [Tor(:, 1); Tor(:, 2)];

[W1, W2] = deal(zeros(n, 9));
for i = 1 : n
    W1(i, :) = [Acc(i, 1), Acc(i, 1)+Acc(i, 2), -d1*(sin(Pos(i, 2))*Vel(i, 2)^2+2*Vel(i, 1)*sin(Pos(i, 2))*Vel(i, 2)-2*Acc(i, 1)*cos(Pos(i, 2))-Acc(i, 2)*cos(Pos(i, 2))), -d1*(cos(Pos(i, 2))*Vel(i, 2)^2+2*Vel(i, 1)*cos(Pos(i, 2))*Vel(i, 2)+2*Acc(i, 1)*sin(Pos(i, 2))+Acc(i, 2)*sin(Pos(i, 2))), 0,  sign(Vel(i, 1)), 0,        Vel(i, 1), 0];
    W2(i, :) = [0,  Acc(i, 1)+Acc(i, 2),  d1*(sin(Pos(i, 2))*Vel(i, 1)^2+Acc(i, 1)*cos(Pos(i, 2))),                               d1*(cos(Pos(i, 2))*Vel(i, 1)^2-Acc(i, 1)*sin(Pos(i, 2))),                              Acc(i, 2), 0,        sign(Vel(i, 2)), 0,  Vel(i, 2)];
end

W = [W1;W2];

theta = inv(W'*W)*W'*torque;


