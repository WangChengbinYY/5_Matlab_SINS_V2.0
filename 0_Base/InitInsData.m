function InsData = InitInsData(G_Const,time,att,vel,pos)
% 初始化 惯导解算结构体数据 
%       坐标系n 北 东 地 xyz 
% 输入：G_Const 常值参数
%       time 采样时刻
%       att[pitch roll yaw]'  姿态 rad
%       vel[v_e v_n v_u]'    速度 m/s
%       pos[lat lon h]'       位置 纬度 经度 高程 ，rad m


%% 输入数据 传感器
    InsData.w_ib_b = zeros(3,1);                    %陀螺输出 角速率
    InsData.f_ib_b = zeros(3,1);                     %加计输出 加速度 b系下投影

%% 输出数据
    InsData.time = time; 
    InsData.att = att;         % x y z   roll pitch yaw
    InsData.vel = vel;
    InsData.pos = pos;
    InsData.posNED = zeros(3,1);    % 以 0为起点，记录 北向  东向  高程 行驶的轨迹
       
%% 中间计算数据
    InsData.q = quaternion([att(3,1),att(2,1),att(1,1)],'euler','ZYX','frame');    % n系转动到b系对应的四元数
    InsData.Cnb = rotmat(InsData.q, 'frame')';

    InsData.Rmh       = EarthRmh(G_Const,pos(1,1),pos(3,1));
    InsData.Rnh        = EarthRnh(G_Const,pos(1,1),pos(3,1));
    % 当地重力值，注意在n系 北东地坐标系下  z朝下，g_n的矢量为[0;0;-g]
    InsData.g             = EarthGn(G_Const,pos(1,1),pos(3,1));     
    InsData.w_ie_n    = EarthWie_n(G_Const,pos(1,1));     % 地球自转角速率在n系下的投影
    InsData.w_en_n   = EarthWen_n(pos(1,1),InsData.vel,InsData.Rmh,InsData.Rnh);   % n系相对e系的转动 在n系下的投影
    InsData.w_in_n    = InsData.w_ie_n+InsData.w_en_n;

    InsData.Delta_Gyro = zeros(3,1);        %当前时刻陀螺输出增量 
    InsData.Delta_Acc   = zeros(3,1);        %当前时刻加计输出增量 
    InsData.DTheta_w_in_n = zeros(3,1); %当前时刻 nxi转动增量
    
    InsData.w_in_n_next = InsData.w_in_n;   %预测下一时刻n系转动矢量，便于计算n系转动增量

    
    
    
    
    
    