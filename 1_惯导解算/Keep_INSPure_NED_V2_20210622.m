%%  *保留*  可作为参考比较的  低精度惯导解算方法  
%   采用了 n系 NED  b系 前右下

clear;clc;
load('D:/IMUGPS2_200_152350.mat');

% 将原始数据的坐标转为  NED
Data_IMU_R = ChangeCoordinate(IMU,1,1);

%% 一、初始化
% 1. 常值初始化
G_Const = InitConst();

% 3. 器件参数设定
% G_IMU.Hz = 200;                         %IMU的采样频率

% 4. 起始姿态、速度、位置设定
% 减去开机起始零偏
GyroBias = mean(Data_IMU_R(1:600,5:7))';
% 设置 初始水平姿态
tmpAcc = -mean(Data_IMU_R(1:200,2:4));
tmpMagnetic = [1 0 0];
q = ecompass(tmpAcc,tmpMagnetic);
if parts(q) < 0
    q = -q;
end
att = euler(q, 'ZYX', 'frame');        

G_Start_Att(1,1) = att(3);   %姿态 横滚角 度
G_Start_Att(2,1) = att(2);   %姿态 俯仰角 度
G_Start_Att(3,1) = deg2rad(10);   %姿态 航向角

G_Start_Vel(1,1) = 0.0;                 %速度 v_n 北向速度
G_Start_Vel(2,1) = 0.0;                 %速度 v_e 东向速度
G_Start_Vel(3,1) = 0.0;                 %速度 v_d 地向速度
G_Start_Pos(1,1) = 34.1 * G_Const.D2R;   %位置 纬度 度
G_Start_Pos(2,1) = 114.1 * G_Const.D2R;   %位置 经度 度
G_Start_Pos(3,1) = 50.0 * G_Const.D2R;   %位置 高程 m

% Bias_Gyro = [7.3177e-04;-8.6012e-04;-4.5023e-04];

%% 二、惯导解算
% 1. 数据准备
[n,m] = size(Data_IMU_R);
Result_AVP = zeros(n,13);               %解算的结果 时间 姿态 速度 位置
Result_AVP(1,1) = Data_IMU_R(1,1);     %时间
Result_AVP(1,2:4) = G_Start_Att';       %姿态
Result_AVP(1,5:7) = G_Start_Vel';       %速度
Result_AVP(1,8:10) = G_Start_Pos';       %位置
Result_AVP(1,11:13) = zeros(3,1); 

INSData_Now = InitInsData(G_Const,Data_IMU_R(1,1),G_Start_Att,G_Start_Vel,G_Start_Pos);
INSData_Pre = INSData_Now;
INSData_Pre.f_ib_b = Data_IMU_R(1,2:4)'.*9.7803267714;

% profile on
% 2. 循环解算
for i=2:n
    Gyro = Data_IMU_R(i,5:7)'-GyroBias;       
    Acc = Data_IMU_R(i,2:4)'.*9.7803267714;
    
    %对加计进行非正交校准
%     K= [0.999844387	0.01471722 	0.002556173;
%         -0.01469017	0.99978481 	0.00053635;
%         -0.01245593	-0.00133620 	1.000481267];
%     M = [0 1 0 ; 1 0 0 ;0 0 -1];
%     Acc = M*K*M*Acc;
%     Gyro = M*K*M*Gyro;
    
    
    INSData_Now = UpdateINS_Low_v1(INSData_Pre,Gyro,Acc,Data_IMU_R(i,1));
   

    % 水平姿态角修正
%     tmpMagnetic = [1 0 0];
%     q = ecompass(-Acc',tmpMagnetic);
%     if parts(q) < 0
%         q = -q;
%     end
%     att = euler(q, 'ZYX', 'frame');     
%     INSData_Now.att(1,1) = att(1,3);
%     INSData_Now.att(2,1) = att(1,2);
%     INSData_Now.q = quaternion([INSData_Now.att(3,1),INSData_Now.att(2,1),INSData_Now.att(1,1)],'euler','ZYX','frame'); 
        
    Result_AVP(i,1) = INSData_Now.time;      
    Result_AVP(i,2:4) = INSData_Now.att';
    Result_AVP(i,5:7) = INSData_Now.vel';        
    Result_AVP(i,8:10) = INSData_Now.pos';
    Result_AVP(i,11:13) = INSData_Now.posNED';
    INSData_Pre =  INSData_Now;
end

    AttResult_INS = zeros(n,4);
    AttResult_INS(:,1) = Data_IMU_R(:,1);
    AttResult_INS(:,2:4) = Result_AVP(:,2:4);
    
    Plot_Att_Group_NED(AttResult_INS);  
    
%  Plot_AVP_Group_NED(Result_AVP);





% Result_AVP_NED = Result_AVP;
% save('D:/ResultAVP_NED.mat','Result_AVP_NED');

% Result_AVP_NED_Cali = Result_AVP;
% save('D:/ResultAVP_NED_Cali.mat','Result_AVP_NED_Cali');

% profile viewer

% Plot_AVP(Result_AVP);


