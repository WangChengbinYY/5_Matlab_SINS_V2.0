%% 比较纯姿态解算 和 惯导解算的区别，就以第一段数据为例

    clear;clc;

    load('D:\N_WorkSpace_GitHub\5_Matlab\5_Matlab_SINS_V2.0\4_姿态解算\1_旋转矩阵姿态误差滤波实验\IMUGPS2_200_152350.mat');
    
    IMU = ChangeCoordinate(IMU,1,1);
    

    Hz = 200;   Ts = 1/Hz;  
    L = length(IMU);
    
    KFTime = 4;   % 滤波周期 
    tmp_Start = [1,         26370,  62360,  97990,  131480, 152151];
    tmp_End =  [14284,  55068,  88150,  121200,  152151];
%-----------------设置零速状态-----------------
    State = zeros(L,2);
    State(:,1) = IMU(:,1);
    for i = 1:5
        State(tmp_Start(i):tmp_End(i),2) = 1;    
    end

% ----------------计算由加计得到的姿态-----------
%     AttResult_Acc = zeros(L,4);
%     AttResult_Acc(:,1) = IMU(:,1);
%     for i = KFTime:L
%         if State(i,2) == 1
%             if State(i-KFTime+1,2) == 1
%                 tp_Acc = -mean(IMU(i-KFTime+1:i,2:4));
%                 tp_Mag = [1,0,0];
%                 tp_q = ecompass(tp_Acc,tp_Mag);
%                 tp_att = euler(tp_q, 'ZYX', 'frame');           
% 
%                 if i == KFTime
%                     AttResult_Acc(1:i,2) = tp_att(1,3); 
%                     AttResult_Acc(1:i,3) = tp_att(1,2);
%                     AttResult_Acc(1:i,4) = deg2rad(10);    % 航向默认不变        
%                 else
%                     AttResult_Acc(i,2) = tp_att(1,3); 
%                     AttResult_Acc(i,3) = tp_att(1,2);
%                     AttResult_Acc(i,4) = deg2rad(10);    % 航向默认不变                  
%                 end     
%             end
%         end
%     end
%     Plot_Att_Group_NED(AttResult_Acc);
%     AttResult_Acc4 = AttResult_Acc;
%      save('D:\N_WorkSpace_GitHub\5_Matlab\5_Matlab_SINS_V2.0\4_姿态解算\1_旋转矩阵姿态误差滤波实验\IMUGPS2_200_152350.mat', ...
%                 'AttResult_Acc4','State','-append');
%=======================================


%% ==================纯姿态解算========================
    Bias = mean(IMU(1:3*Hz,5:7))';
    
    AttResult_Gyro = zeros(L,4);
    AttResult_Gyro(1:L,1) = IMU(1:L,1);           
    
    % 计算起始姿态 1s 加计数据
        tp_Acc = -mean(IMU(1:1*Hz,2:4));
        tp_Mag = [1,0,0]; 
        tp_q = ecompass(tp_Acc,tp_Mag);
        tp_att = euler(tp_q, 'ZYX', 'frame'); 
        tp_Att0(1,1) = tp_att(1,3);         %  起始姿态 欧拉角  x横滚 y俯仰 z航向  前右下
        tp_Att0(2,1) = tp_att(1,2);
        tp_Att0(3,1) = deg2rad(10);              

        Att_pre.Euler = tp_Att0;   % 起始姿态 欧拉角  x横滚 y俯仰 z航向  前右下
        Att_pre.Q = quaternion([tp_Att0(3),tp_Att0(2),tp_Att0(1)],'euler','ZYX','frame');
        Att_pre.Cnb = rotmat(Att_pre.Q, 'frame')';
        Att_pre.Time = 1;
        Att_pre.Gyro = zeros(3,1);      

        for i = 1:L
            tmp_Gyro = IMU(i,5:7)'-Bias;
            Att_now = UpdateAtt_GyroLow_v1(i,Att_pre,tmp_Gyro,Hz);
            Att_pre = Att_now;
            AttResult_Gyro(i,2:4) = Att_now.Euler';       
        end
        
     save('D:\N_WorkSpace_GitHub\5_Matlab\5_Matlab_SINS_V2.0\4_姿态解算\1_旋转矩阵姿态误差滤波实验\IMUGPS2_200_152350.mat', ...
     'AttResult_Gyro','-append');        
 
        Plot_Att_Group_NED(AttResult_INS,AttResult_Gyro);  

        
%% ==================惯导解算========================        
        
        
        
        
            
            
            
