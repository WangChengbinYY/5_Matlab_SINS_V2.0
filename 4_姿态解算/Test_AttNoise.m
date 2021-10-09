%% 器件白噪声对姿态解算的影响

%% ADI 器件噪声程度分析
    clear; clc;
    % 噪声 程度 1.5e-3 ~ 3e-3 弧度/s  解算3分钟，200Hz
    Hz = 200;
    L = 3*60*Hz;
    
    % ADI 陀螺噪声方差 x轴3e-7 y轴4e-7 z轴3e-7
    % Mpu9250 陀螺噪声方差 x轴1e-5 y轴1e-6 z轴1e-6
    Gyro = zeros(L,3);
    
%     Gyro(:,1) = wgn(L,1,3e-7,'linear');
%     Gyro(:,2) = wgn(L,1,4e-7,'linear');
%     Gyro(:,3) = wgn(L,1,3e-7,'linear');

%     Gyro(:,1) = ones(L,1).*(15*pi/180/3600*cos(deg2rad(34))) ;
%     Gyro(:,2) = ones(L,1).*(15*pi/180/3600*cos(deg2rad(34))) ;
%     Gyro(:,3) = ones(L,1).*(-15*pi/180/3600*sin(deg2rad(34))) ;

    Noise_x = wgn(L,1,1e-7,'linear');
    Noise_y = wgn(L,1,1e-7,'linear');
    Noise_z = wgn(L,1,1e-7,'linear');
    
    tmp_Att = zeros(L,4);
    tmp_Att(:,1) = (1:L);
    tmp_Att(:,2) = cumsum(Noise_x).*(0.005);
    tmp_Att(:,3) = cumsum(Noise_y).*(0.005);
    tmp_Att(:,4) = cumsum(Noise_z).*(0.005);

    Gyro(:,1) = ones(L,1).*(15*pi/180/3600*cos(deg2rad(34))) +Noise_x; 
    Gyro(:,2) = ones(L,1).*(15*pi/180/3600*cos(deg2rad(34))) +Noise_y;
    Gyro(:,3) = ones(L,1).*(-15*pi/180/3600*sin(deg2rad(34))) +Noise_z;
    




    Att_pre.Euler = deg2rad([0,0,0]');   % 起始姿态 欧拉角  x横滚 y俯仰 z航向  前右下
    Att_pre.Q = quaternion(Att_pre.Euler','euler','ZYX','frame');
    Att_pre.Time = 1;
    Att_pre.Gyro = zeros(3,1);  
    
    Att_now = Att_pre;
    Att_result = zeros(L,4);
    
     Att_result(1,1) = 1;
    Att_result(1,2:4) = Att_pre.Euler;
    
    for i = 2:L
        tmp_Gyro = Gyro(i,:)';
        
        Att_now = UpdateAtt_GyroLow_v1(i,Att_pre,tmp_Gyro,Hz);
        Att_pre = Att_now;

         Att_result(i,1) = i;
        Att_result(i,2:4) = Att_now.Euler';
    end
    
%     Plot_Att_Group_NED(Att_result);
tmp_new = tmp_Att;
tmp_new(:,2:4) = tmp_new(:,2:4) + Att_result_old(:,2:4);

     Plot_Att_Group_NED(Att_result_old,tmp_new);
    
    