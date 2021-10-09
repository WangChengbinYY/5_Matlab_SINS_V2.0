function INSData = UpdateINS_Low_v1(INSData_Pre,Gyro,Acc,T)
% 纯惯性解算，时间更新
%       输入：
%               INSData_pre 上一时刻的惯导结构数据
%               Gyro 当前时刻陀螺传感器数据  xyz   单位 rad/s
%               Acc 当前时刻加速度计传感器数据 xyz 单位 m/s
%               T 当前时间 单位 s
%       输出：
%               INSData 计算后的惯导结构数据
%
%       版本：
%               v1：低成本惯导应用


%% 一、 计算中间量
    % 保存当前时刻传感器数据
        INSData.w_ib_b = Gyro;
        INSData.f_ib_b = Acc;
    % 时间间隔
        INSData.time = T;   
        Delta_T = T - INSData_Pre.time;  %采样间隔时间 可以直接利用采样频率计算，也可以利用采样间隔计算(防止丢包)           
    % 地球参数
        Rmh = INSData_Pre.Rmh;               % 简易计算
        Rnh = INSData_Pre.Rnh;                  % 简易计算
        w_ie_n = INSData_Pre.w_ie_n;         % 简易计算
        w_en_n = INSData_Pre.w_en_n;       % 简易计算 
    % 当前时刻陀螺和加计的增量  利用上一时刻和当前时刻进行计算
        Delta_Gyro = (Gyro+INSData_Pre.w_ib_b) * Delta_T * 0.5;
        Delta_Acc = (Acc+INSData_Pre.f_ib_b) * Delta_T * 0.5;        
    % 上一时刻陀螺和加计的增量，便于书写
        Delta_Gyro_Pre = INSData_Pre.Delta_Gyro;
        Delta_Acc_Pre = INSData_Pre.Delta_Acc;        
    % 当前时刻 n系 转动增量 
        DTheta_w_in_n = INSData_Pre.w_in_n_next * Delta_T;      % 此处未依据速度更新，而是简单预测
    % 上一时刻 n系 的转动增量
        DTheta_w_in_n_Pre = INSData_Pre.DTheta_w_in_n;
    

%% 二、姿态更新
    % Q_n(m)_b(m) = Q_n(m)_n(m-1) * Q_n(m-1)_b(m-1) * Q_b(m-1)_b(m)
    % Q_n(m)_b(m) = Q1                     *  Q2                         *Q3
    % 计算 Q_b(m-1)_b(m)        
        tmp_vector = Delta_Gyro + cross(Delta_Gyro_Pre,Delta_Gyro)./12;
        Q3 = quaternion(tmp_vector','rotvec');
    % 计算Q_n(m-1)_n(m)
        tmp_vector = DTheta_w_in_n + cross(DTheta_w_in_n_Pre,DTheta_w_in_n)./12;
        tmp = quaternion(tmp_vector','rotvec');
        Q1 = conj(tmp);
    % 前一时刻的四元数 Q_n(m-1)_b(m-1)
        Q2 = INSData_Pre.q;
    % 计算当前时刻的四元数
        Q_nb_m = Q1 * Q2 * Q3;
    % 计算导航输出的 欧拉角
        INSData.q = Q_nb_m;
        INSData.Cnb = rotmat(Q_nb_m, 'frame')';
        tmp_Euler = euler(Q_nb_m, 'ZYX', 'frame');
        INSData.att(1,1) = tmp_Euler(1,3);
        INSData.att(2,1) = tmp_Euler(1,2);
        INSData.att(3,1) = tmp_Euler(1,1);
        
%% 三、速度更新
    % 计算 增量  Delta_v_rot  
        Delta_v_rot = cross(Delta_Gyro,Delta_Acc)./2;
    % 计算 增量 Delta_v_scul
        Delta_v_scul = (cross(Delta_Gyro_Pre,Delta_Acc) + cross(Delta_Acc_Pre,Delta_Gyro))./12;
    % 计算 增量 Delta_v_sf
        tmp = eye(3) - AskewofVector(INSData_Pre.w_in_n).*(Delta_T/2);
        Delta_v_sf = tmp * INSData.Cnb * (Delta_Acc + Delta_v_rot + Delta_v_scul);
        
    % 计算 增量 Delta_v_cor
        g_n = [0;0;9.7803267714];
        tmp = -(w_ie_n.*2 + w_en_n);
        Delta_v_cor = (cross(tmp,INSData_Pre.vel) + g_n).*Delta_T;
     
    % 计算当前时刻的速度
        INSData.vel = INSData_Pre.vel + Delta_v_sf + Delta_v_cor;
        
%% 四、位置更新
    % 计算当前时刻平均速度
        tmp_vel = (INSData_Pre.vel + INSData.vel).*0.5;
    % 计算当前时刻经纬度位置坐标
        INSData.pos(1,1) = INSData_Pre.pos(1,1) + tmp_vel(1,1)/Rmh*Delta_T;
        INSData.pos(2,1) = INSData_Pre.pos(2,1) + tmp_vel(2,1)/Rnh*sec(INSData.pos(1,1))*Delta_T;
        INSData.pos(3,1) = INSData_Pre.pos(3,1) - tmp_vel(3,1)*Delta_T;
    % 计算当前时刻北向 东向 高程  坐标 单位 m  (便于绘图比较)
        INSData.posNED(1,1) = INSData_Pre.posNED(1,1) + tmp_vel(1,1)*Delta_T;
        INSData.posNED(2,1) = INSData_Pre.posNED(2,1) + tmp_vel(2,1)*Delta_T;
        INSData.posNED(3,1) = INSData_Pre.posNED(3,1) - tmp_vel(3,1)*Delta_T;        
        
%% 更新
    % 更新地球参数(简易计算)
        INSData.Rmh        = Rmh;       
        INSData.Rnh         = Rnh;    
        INSData.g             = INSData_Pre.g;   
        INSData.w_ie_n    = w_ie_n;    
        INSData.w_en_n   = EarthWen_n(INSData.pos(1,1),INSData.vel,INSData.Rmh,INSData.Rnh);   % n系相对e系的转动 在n系下的投影
        INSData.w_in_n    = w_ie_n+INSData.w_en_n;
        % 预测下一(m+1/2)时刻的n系转动角速度，便于n系转动增量的计算
        INSData.w_in_n_next = INSData_Pre.w_in_n + (INSData.w_in_n - INSData_Pre.w_in_n) * 1.5;
        
    % 更新当前时刻 陀螺和加计增量
        INSData.Delta_Gyro = Delta_Gyro;
        INSData.Delta_Acc = Delta_Acc;   

    % 更新当前时刻 n系的转动增量
        INSData.DTheta_w_in_n = (INSData_Pre.w_in_n + INSData.w_in_n) * Delta_T * 0.5;
     
   

    
