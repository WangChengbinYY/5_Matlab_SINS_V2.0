function Att_now = UpdateAtt_GyroLow_v1(Time,Att_pre,Gyro,Hz)
% 仅依靠陀螺进行的姿态更新，低精度陀螺，暂不考虑地球自转角速度的影响,15度/h
%       输入：
%               Time        当前时刻
%               Att_pre     上一时刻的姿态相关信息
%               Gyro        当前时刻陀螺的输出  前右下 弧度/s
%       输出：
%               Att_Now  计算后的姿态信息
%
%       版本：
%               v1：低成本惯导应用


Att_now.Time = Time;
Att_now.Gyro = Gyro;
DTime = 1/Hz;
% 上一时刻的陀螺角增量输出

DTheta_pre = (Att_pre.Gyro + Gyro).*(DTime/2);
DTheta_now = (Gyro.*1.5 -  Att_pre.Gyro.*0.5).*(DTime);

tmp_vector = DTheta_now + cross(DTheta_pre,DTheta_now)./12;
tmp_Q = quaternion(tmp_vector','rotvec');

Att_now.Q = Att_pre.Q * tmp_Q;
Att_now.Cnb = rotmat(Att_now.Q, 'frame')';
tmp_att = euler(Att_now.Q,'ZYX','frame');

Att_now.Euler(1,1) = tmp_att(1,3);
Att_now.Euler(2,1) = tmp_att(1,2);
Att_now.Euler(3,1) = tmp_att(1,1);




