%%  *保留*  可作为参考比较的  低精度惯导解算方法  
%   采用了 n系 NED  b系 前右下 但是输入输出均为 ENU

clear;clc;
load('D:/IMUGPS2_200_152350.mat');

Data_IMU_R = ChangeCoordinate(IMU,1,1);

%% 一、初始化
% 1. 常值初始化
G_Const = InitConst();

% 3. 器件参数设定
G_IMU.Hz = 200;                         %IMU的采样频率

% 4. 起始姿态、速度、位置设定
G_Start_Att(1,1) = deg2rad(5);   %姿态 横滚角 度
G_Start_Att(2,1) = deg2rad(10);   %姿态 俯仰角 度
G_Start_Att(3,1) = deg2rad(20);   %姿态 航向角 (北偏西为正！！)
G_Start_Vel(1,1) = 100.0;                 %速度 v_n 北向速度
G_Start_Vel(2,1) = 200.0;                 %速度 v_e 东向速度
G_Start_Vel(3,1) = -50.0;                 %速度 v_d 地向速度
G_Start_Pos(1,1) = 34.1 * G_Const.D2R;   %位置 纬度 度
G_Start_Pos(2,1) = 114.1 * G_Const.D2R;   %位置 经度 度
G_Start_Pos(3,1) = 50.0 * G_Const.D2R;   %位置 高程 m

% Bias_Gyro = [7.3177e-04;-8.6012e-04;-4.5023e-04];

%% 二、惯导解算
% 1. 数据准备
[n,m] = size(Data_IMU_R);
Result_AVP = zeros(n,10);               %解算的结果 时间 姿态 速度 位置
Result_AVP(1,1) = Data_IMU_R(1,1);     %时间
% Result_AVP(1,2:4) = G_Start_Att';       %姿态
Result_AVP(1,2) = G_Start_Att(2,1);
Result_AVP(1,3) = G_Start_Att(1,1);
Result_AVP(1,4) = G_Start_Att(3,1);

% Result_AVP(1,5:7) = G_Start_Vel';       %速度
Result_AVP(1,5) = G_Start_Vel(2,1);
Result_AVP(1,6) = G_Start_Vel(1,1);
Result_AVP(1,7) = G_Start_Vel(3,1);


Result_AVP(1,8:10) = G_Start_Pos';       %位置

INSData_Now = InitInsData(G_Const,Data_IMU_R(1,1),G_Start_Att,G_Start_Vel,G_Start_Pos);
INSData_Pre = INSData_Now;

% profile on
% 2. 循环解算
for i=2:n
    Gyro = Data_IMU_R(i,5:7)';       
%     Acc = Data_IMU_R(i,2:4)'.*9.7803267714;
    Acc = [10;30;-20];
    INSData_Now = UpdateINS_Low_v1(INSData_Pre,Gyro,Acc,Data_IMU_R(i,1));
    
    if (INSData_Now.vel(1,1)^2+INSData_Now.vel(2,1)^2+INSData_Now.vel(3,1)^2) > 10
        INSData_Now.vel = zeros(3,1);
    end
    
    
    Result_AVP(i,1) = INSData_Now.time;      
    Result_AVP(i,2) = INSData_Now.att(2,1);
    Result_AVP(i,3) = INSData_Now.att(1,1);
    Result_AVP(i,4) = INSData_Now.att(3,1);

    Result_AVP(i,5) = INSData_Now.vel(2,1);
    Result_AVP(i,6) = INSData_Now.vel(1,1);
    Result_AVP(i,7) = -INSData_Now.vel(3,1);
        
    Result_AVP(i,8:10) = INSData_Now.pos';
  
    INSData_Pre =  INSData_Now;
end

Result_AVP_new = Result_AVP;
save('D:/newResultAVP.mat','Result_AVP_new');

% profile viewer

% Plot_AVP(Result_AVP);


