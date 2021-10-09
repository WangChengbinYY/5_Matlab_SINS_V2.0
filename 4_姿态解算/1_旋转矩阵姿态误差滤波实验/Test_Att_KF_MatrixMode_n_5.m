%% 1.测试真实数据 加计姿态对陀螺的估计，针对姿态误差模型的KF滤波实验，
%   2.基于n系的误差矩阵
%   3.静止状态下进行速度约束，直接归零 速度归零，速度增量归0

clear;clc;

load('D:\N_WorkSpace_GitHub\5_Matlab\5_Matlab_SINS_V2.0\4_姿态解算\1_旋转矩阵姿态误差滤波实验\IMUGPS2_200_152350.mat');

IMU = ChangeCoordinate(IMU,1,1);

%% 修改数据，将所有静止数据连在一起
%     tmp_Start = [1,         26370,  62360,  97990,  131480];
%     tmp_End =  [14284,  55068,  88150,  121200,  152151];

%     j = 1;
%     for i = 1:5
%         tmp_L = tmp_End(i) - tmp_Start(i) + 1;
%         tmp_IMU(j:j+tmp_L-1,:) = IMU(tmp_Start(i):tmp_End(i),:);
%         j = j + tmp_L;
%     end
% IMU = tmp_IMU;


Hz = 200;  Ts = 1/Hz;  
L = length(IMU);
L = 55068;
% L = 14284;

KFTime = 4;   % 滤波周期 
%% 一、解算初始化
    % 1. 常值初始化
        G_Const = InitConst();
    % 初始零偏
        Bias = mean(IMU(1:3*Hz,5:7))';

    % 计算起始姿态 1s 加计数据
        tp_Acc = -mean(IMU(1:1*Hz,2:4));
        tp_Mag = [1,0,0]; 
        tp_q = ecompass(tp_Acc,tp_Mag);
        tp_att = euler(tp_q, 'ZYX', 'frame'); 
        G_Start_Att(1,1) = tp_att(3);   %起始姿态 欧拉角  x横滚 y俯仰 z航向  前右下
        G_Start_Att(2,1) = tp_att(2);   %姿态 俯仰角 度
        G_Start_Att(3,1) = deg2rad(10);   %姿态 航向角
            
        G_Start_Vel(1,1) = 0.0;                 %速度 v_n 北向速度
        G_Start_Vel(2,1) = 0.0;                 %速度 v_e 东向速度
        G_Start_Vel(3,1) = 0.0;                 %速度 v_d 地向速度
        G_Start_Pos(1,1) = 34.1 * G_Const.D2R;   %位置 纬度 度
        G_Start_Pos(2,1) = 114.1 * G_Const.D2R;   %位置 经度 度
        G_Start_Pos(3,1) = 50.0 * G_Const.D2R;   %位置 高程 m
        
%% 二、惯导解算     
    % 惯导解算数据结构初始化
        INSData_Now = InitInsData(G_Const,IMU(1,1),G_Start_Att,G_Start_Vel,G_Start_Pos);
        INSData_Pre = INSData_Now;
        INSData_Pre.f_ib_b = IMU(1,2:4)'.*9.7803267714;
    
    % 数据存储
        Result_AVP = zeros(L,13);               %解算的结果 时间 姿态 速度 位置
        Result_AVP(1,1) = IMU(1,1);     %时间
        Result_AVP(1,2:4) = G_Start_Att';       %姿态
        Result_AVP(1,5:7) = G_Start_Vel';       %速度
        Result_AVP(1,8:10) = G_Start_Pos';       %位置
        Result_AVP(1,11:13) = zeros(3,1);   
        
        Att_result_KF_Acc = zeros(L,4);
        Att_result_KF_Acc(:,1) = IMU(1:L,1);
        Att_result_KF_Acc(1,2:4) = INSData_Now.att;
        
        Bias_result_KF_Acc = zeros(L,4);
        Bias_result_KF_Acc(:,1) = IMU(1:L,1);
        Bias_result_KF_Acc(1,2:4) = Bias';
        
        XkPk = zeros(fix(L/KFTime),13);  j = 1;
        Xk_1Pk_1 = zeros(fix(L/KFTime),13); j_1 = 1;
    %KF初始化
        mode = 1; n = 6; k = 3; m = 3; 
        KF = InitKF(1,6,3,3);
        % 初始状态量及方差
            X0 = [deg2rad([0.1;0.1;0.2;]);zeros(3,1)]; 
            P0 = diag(X0.^2);
            KF.Xk = X0;   KF.Pk = P0;

        % 状态噪声方差设置
            Q0 = diag([1.7e-7,1.4e-7,1.7e-7]);
            R0 = diag([1.7e-5,1.4e-5,1.7e-5]);
            KF.Qk = Q0;     KF.Rk = R0; 
            Gt = [zeros(3,3);eye(3)];
        % 反馈系数
            v = 1;
 %===============循环解算==============       
    for i = 1:L
        % 依据 加计 陀螺 数据导航更新
            Gyro = IMU(i,5:7)'-Bias;       
            Acc = IMU(i,2:4)'.*9.7803267714;
            
        %对加计进行非正交校准
            K= [0.999844387	0.01471722 	0.002556173;
                -0.01469017	0.99978481 	0.00053635;
                -0.01245593	-0.00133620 	1.000481267];
            M = [0 1 0 ; 1 0 0 ;0 0 -1];
            Acc = M*K*M*Acc;
            
        % 导航更新
            INSData_Now = UpdateINS_Low_v1(INSData_Pre,Gyro,Acc,IMU(i,1));
            
        % KF-------- 时间更新
            % 更新F(t)矩阵
                Ft = [zeros(6,3),[-INSData_Now.Cnb;zeros(3,3)]];
            % 更新Phi  Gk 矩阵
                KF.Phikk_1 = eye(n) + Ft.*Ts + Ft^2.*(Ts^2/2);
                KF.Gk = (Gt+Ft*Gt.*(Ts/2)).*Ts;
            % 更新系统状态量    
                KF.Xkk_1 = KF.Phikk_1*KF.Xk;
                KF.Pkk_1 = KF.Phikk_1*KF.Pk*KF.Phikk_1'+KF.Gk*KF.Qk*KF.Gk';
                
                Xk_1Pk_1(j_1,1) = IMU(i,1);   %时间
                Xk_1Pk_1(j_1,2:7) =KF.Xkk_1';
                Xk_1Pk_1(j_1,8:13) = (diag(KF.Pkk_1))';                            
                j_1 = j_1 +1;    
        % KF-------- 量测更新   
                if (mod(i,KFTime) == 0) && (State(i,2) == 1)
                    % 获取观测量Z
                        tmp_Att = AttResult_Acc4(i,2:4)';                   
                        tmp_Cnb = AttChange_E2Mnb(tmp_Att);
                        tmp_Cnn = INSData_Now.Cnb*tmp_Cnb';
                        tmp_q = quaternion(tmp_Cnn,'rotmat','frame');
                        KF.Zk = rotvec(tmp_q)';            
                        KF.Kk = KF.Pkk_1*KF.Hk'*(KF.Hk*KF.Pkk_1*KF.Hk'+KF.Rk)^-1;
                        KF.Xk = KF.Xkk_1+KF.Kk*(KF.Zk-KF.Hk*KF.Xkk_1);
                        KF.Pk = (eye(n)-KF.Kk*KF.Hk)*KF.Pkk_1*(eye(n)-KF.Kk*KF.Hk)'+KF.Kk*KF.Rk*KF.Kk';    
                        
                        % 中间状态存储
                        XkPk(j,1) = IMU(i,1);   %时间
                        XkPk(j,2:7) =KF.Xk';
                        XkPk(j,8:13) = (diag(KF.Pk))';                     
                        j = j + 1; 
                        
                    % 状态反馈
                        % 姿态修正
                        tmp_Att = KF.Xk(1:3,1);
                        tmp_Cnn = eye(3)-AskewofVector(tmp_Att);
                        INSData_Now.Cnb = tmp_Cnn'*INSData_Now.Cnb;
                        INSData_Now.q = quaternion(INSData_Now.Cnb','rotmat','frame');
                        tmp_Euler = euler(INSData_Now.q, 'ZYX', 'frame');
                        INSData_Now.att(1,1) = tmp_Euler(1,3);
                        INSData_Now.att(2,1) = tmp_Euler(1,2);
                        INSData_Now.att(3,1) = tmp_Euler(1,1);
                        KF.Xk(1:3,1) = zeros(3,1);
                        % 零偏修正
                        Bias = Bias + (INSData_Now.Cnb'*KF.Xk(4:6,1)).*(v);
                        KF.Xk(4:6,1) = KF.Xk(4:6,1).*(1-v);                        
                        
                    %速度修正
%                         INSData_Now.vel = zeros(3,1);


                else
                    KF.Xk = KF.Xkk_1;
                    KF.Pk = KF.Pkk_1;              
                end            
                INSData_Pre =  INSData_Now;
        % 保存结果数据      
                Result_AVP(i,1) = INSData_Now.time;      
                Result_AVP(i,2:4) = INSData_Now.att';
                Result_AVP(i,5:7) = INSData_Now.vel';        
                Result_AVP(i,8:10) = INSData_Now.pos';
                Result_AVP(i,11:13) = INSData_Now.posNED';
                
                Att_result_KF_Acc(i,2:4) = INSData_Now.att';
                Bias_result_KF_Acc(i,2:4) = Bias';
    end    
       
%      save('D:\N_WorkSpace_GitHub\5_Matlab\5_Matlab_SINS_V2.0\4_姿态解算\1_旋转矩阵姿态误差滤波实验\IMUGPS2_200_152350.mat', ...
%      'Bias_result_KF_Acc','Att_result_KF_Acc','-append');
     

    
%     Plot_AVP_Group_NED(Result_AVP);
    
%      Plot_Att_Group_NED(AttResult_Acc4,Att_result_KF_Acc);     
%      Plot_GyroBias(Bias_result_KF_Acc);




