%% 在基础版本上，模拟中间运动停止滤波的情况

clear;clc;

load('D:\N_WorkSpace_GitHub\5_Matlab\5_Matlab_SINS_V2.0\4_姿态解算\1_旋转矩阵姿态误差滤波实验\SimulationData.mat');


Hz = 200;
L = length(Gyro_true);

% 设计个间断标志位  原数据应该是180秒 50s后，每隔3秒断1秒
% Label = ones(1,L);
% b = (54:3:180);
% tmp_L = length(b);
% for i=1:tmp_L
%     tp_start = (b(i)-1)*200+1;
%     tp_end = b(i)*200;
%     Label(tp_start:tp_end) = 0;
% end

% 50s后，每隔1秒断3秒
Label = zeros(1,L);  Label (1:50*200) = 1;
b = (54:3:180);
tmp_L = length(b);
for i=1:tmp_L
    tp_start = (b(i)-1)*200+1;
    tp_end = b(i)*200;
    Label(tp_start:tp_end) = 1;
end


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
%------------------KF初始化-------------------
    mode = 1; n = 6; k = 3; m = 3; Ts = 1/Hz;
    KF = InitKF(1,6,3,3);
    % 初始状态量及方差
        X0 = [deg2rad([0.5;0.5;2;]);Bias]; 
        P0 = diag(X0.^2);
        KF.Xk = X0;   KF.Pk = P0;
        
    % 状态噪声方差设置
        Q0 = diag([4.4e-9 2.2e-8 1.8e-8]);
        R0 = diag([1.7e-6 1.7e-6 1.7e-6]);
        KF.Qk = Q0;     KF.Rk = R0; 
        Gt = [zeros(3,3);eye(3)];
    % 反馈系数
        v = 1;
 %===============循环解算==============       
    for i = 1:L
        % 依据陀螺数据进行姿态更新
            tmp_Gyro = Gyro_noise(i,:)'-Bias;
            Att_now = UpdateAtt_GyroLow_v1(i,Att_pre,tmp_Gyro,Hz);
            
            
        % KF-------- 时间更新
            % 更新F(t)矩阵
                Ft = [zeros(6,3),[-Att_now.Cnb;zeros(3,3)]];
            % 更新Phi  Gk 矩阵
                KF.Phikk_1 = eye(n) + Ft.*Ts + Ft^2.*(Ts^2/2);
                KF.Gk = (Gt+Ft*Gt.*(Ts/2)).*Ts;
            % 更新系统状态量    
                KF.Xkk_1 = KF.Phikk_1*KF.Xk;
                KF.Pkk_1 = KF.Phikk_1*KF.Pk*KF.Phikk_1'+KF.Gk*KF.Qk*KF.Gk';
        % KF-------- 量测更新   
%                 if mod(i,4) == 0 && Label(i) ==1
                if mod(i,4) == 0 
                    % 获取观测量Z
                        tmp_Att = AttResult_z(i,2:4)';
                        tmp_Cnb = AttChange_E2Mnb(tmp_Att);
                        tmp_Cnn = Att_now.Cnb*tmp_Cnb';
                        tmp_q = quaternion(tmp_Cnn,'rotmat','frame');
                        KF.Zk = rotvec(tmp_q)';                        
                        
                        KF.Kk = KF.Pkk_1*KF.Hk'*(KF.Hk*KF.Pkk_1*KF.Hk'+KF.Rk)^-1;
                        KF.Xk = KF.Xkk_1+KF.Kk*(KF.Zk-KF.Hk*KF.Xkk_1);
                        KF.Pk = (eye(n)-KF.Kk*KF.Hk)*KF.Pkk_1*(eye(n)-KF.Kk*KF.Hk)'+KF.Kk*KF.Rk*KF.Kk';    
                        
                    % 状态反馈
                        % 姿态修正
                        tmp_Att = KF.Xk(1:3,1);
                        tmp_Cnn = eye(3)-AskewofVector(tmp_Att);
                        Att_now.Cnb = tmp_Cnn'*Att_now.Cnb;
                        Att_now.Q = quaternion(Att_now.Cnb','rotmat','frame');
                        tmp_Euler = euler(Att_now.Q, 'ZYX', 'frame');
                        Att_now.Euler(1,1) = tmp_Euler(1,3);
                        Att_now.Euler(2,1) = tmp_Euler(1,2);
                        Att_now.Euler(3,1) = tmp_Euler(1,1);
                        KF.Xk(1:3,1) = zeros(3,1);
                        % 零偏修正
                        Bias = Bias + (Att_now.Cnb'*KF.Xk(4:6,1)).*(v);
                        KF.Xk(4:6,1) = KF.Xk(4:6,1).*(1-v);   
                        
                else
                    KF.Xk = KF.Xkk_1;
                    KF.Pk = KF.Pkk_1;
                end            
                Att_pre = Att_now;
        % 保存结果数据      
                Att_result(i,2:4) = Att_now.Euler';
                Bias_result(i,2:4) = Bias';
    end    
       
      Plot_GyroBias(Bias_result,Bias_result_duan);    

     





