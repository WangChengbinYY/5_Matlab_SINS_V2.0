%% 制作仿真数据
clear;clc;
% 时间长度  3分钟
% 采样频率 200Hz
Hz = 200;
L = 3*60*Hz;

Noise_x = wgn(L,1,1e-7,'linear');
Noise_y = wgn(L,1,1e-7,'linear');
Noise_z = wgn(L,1,1e-7,'linear');

Bias_true = zeros(L,4);
Bias_true(:,1) =(1:L)./Hz;
Bias_true(:,2) = cumsum(Noise_x).*(0.005)  + 4e-4;
Bias_true(:,3) = cumsum(Noise_y).*(0.005)  + 4e-4;
Bias_true(:,4) = cumsum(Noise_z).*(0.005)  + 4e-4;

Gyro_true = zeros(L,3);
Gyro_true(:,1) = ones(L,1).*(1500*pi/180/3600*cos(deg2rad(34))); 
Gyro_true(:,2) = ones(L,1).*(1500*pi/180/3600*cos(deg2rad(34)));
Gyro_true(:,3) = ones(L,1).*(-1500*pi/180/3600*sin(deg2rad(34)));

% 制作有噪声的陀螺输出
Gyro_noise = Gyro_true +BiasGyro;

% 计算真实姿态数据
    Att0_true = deg2rad([3;5;15]);    %  起始姿态 欧拉角  x横滚 y俯仰 z航向  前右下
    Att_pre.Euler = Att0_true;   % 起始姿态 欧拉角  x横滚 y俯仰 z航向  前右下
    Att_pre.Q = quaternion([Att0_true(3),Att0_true(2),Att0_true(1)],'euler','ZYX','frame');
    Att_pre.Time = 1;
    Att_pre.Gyro = zeros(3,1);  

    Att_now = Att_pre;
    Att_result = zeros(L,4);
    Att_result(1,1) = 1/Hz;
    Att_result(1,2:4) = Att_pre.Euler;

   for i = 2:L
        tmp_Gyro = Gyro_noise(i,:)';
        
        Att_now = UpdateAtt_GyroLow_v1(i,Att_pre,tmp_Gyro,Hz);
        Att_pre = Att_now;

        Att_result(i,1) = i/Hz;
        Att_result(i,2:4) = Att_now.Euler';
   end    
    Plot_Att_Group_NED(AttResult_true,Att_result);
    
    
    % 制作有噪声的姿态数据
    
Noise_x = wgn(L,1,deg2rad(0.0001),'linear');
Noise_y = wgn(L,1,deg2rad(0.0001),'linear');
Noise_z = wgn(L,1,deg2rad(0.0001),'linear');    
AttResult_z = Att_result;
AttResult_z(:,2:4) = Att_result(:,2:4)+[Noise_x,Noise_y,Noise_z];
Plot_Att_Group_NED(AttResult_z,AttResult_true);
    AttResult_noise = Att_result;

 save('SimulationData.mat','Bias_true','Att0_true','Gyro_true','AttResult_true','Gyro_noise','AttResult_z','AttResult_noise');
 

 