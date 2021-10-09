%% 简单粗暴的用失准角直接求取零偏测试 实验结论不靠谱

clear;clc;

load('D:\N_WorkSpace_GitHub\5_Matlab\5_Matlab_SINS_V2.0\4_姿态解算\1_旋转矩阵姿态误差滤波实验\SimulationData.mat');


Hz = 200;  Ts = 1/Hz;
L = length(Gyro_true);



%------------------姿态解算初始化-------------------
    Att0_noise = deg2rad([3+0.5;5+0.5;15+2]);    %  起始姿态 欧拉角  x横滚 y俯仰 z航向  前右下
    Att_pre.Euler = Att0_noise;   % 起始姿态 欧拉角  x横滚 y俯仰 z航向  前右下
    Att_pre.Q = quaternion([Att0_noise(3),Att0_noise(2),Att0_noise(1)],'euler','ZYX','frame');
    Att_pre.Cnb = rotmat(Att_pre.Q, 'frame')';
    Att_pre.Time = 1;
    Att_pre.Gyro = zeros(3,1);  

    Att_now = Att_pre;

    % 初始零偏
    Bias = [4e-3;4e-3;4e-3];
    
    
    % 数据存储
    Att_result = zeros(L,4);
    Att_result(:,1) = (1:L)/200;
    Att_result(1,2:4) = Att_pre.Euler;
    Bias_result = zeros(L,4);
    Bias_result(:,1) = (1:L)/200;
    Bias_result(1,2:4) = Bias';

 %===============循环解算==============       
    for i = 1:L
        % 依据陀螺数据进行姿态更新
            tmp_Gyro = Gyro_noise(i,:)'-Bias;
            Att_now = UpdateAtt_GyroLow_v1(i,Att_pre,tmp_Gyro,Hz);
            
                if mod(i,4) == 0
                    % 获取观测量Z
                        tmp_Att = AttResult_z(i,2:4)';
                        tmp_Cnb = AttChange_E2Mnb(tmp_Att);
                        tmp_Cnn = Att_now.Cnb*tmp_Cnb';
                        tmp_q = quaternion(tmp_Cnn,'rotmat','frame');
                        KF.Zk = rotvec(tmp_q)';                        
                        
                        Bias = Bias - KF.Zk./(Ts*4);
                        
                    % 状态反馈
                        % 姿态修正
                        Att_now.Euler = tmp_Att;
                        Att_now.Q = quaternion([tmp_Att(3,1),tmp_Att(2,1),tmp_Att(1,1)],'euler','ZYX','frame');
                        Att_now.Cnb = rotmat(Att_now.Q, 'frame')';

                end            
                Att_pre = Att_now;
                
        % 保存结果数据      
                Att_result(i,2:4) = Att_now.Euler';
                Bias_result(i,2:4) = Bias';
    end    
       
     
     Plot_Att_Group_NED(AttResult_z,Att_result);
     
    Plot_GyroBias(Bias_true,Bias_result);




