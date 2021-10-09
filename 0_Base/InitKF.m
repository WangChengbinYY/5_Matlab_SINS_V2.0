function KF = InitKF(mode,n,k,m)
% 初始化KF滤波数据结构体
%       系统状态方程及观测方程
%           Xk = Phikk_1*Xk_1 + Gk*Wk   Wk的方差为Qk
%           Zk = Hk*Xk + Vk                   Vk的方差为Rk
%       n:  状态量维数   k: 状态白噪声维数  m: 观测量维数
%       mode
%               1:  表示6纬度的姿态滤波，3个失准角及3个陀螺零偏

KF = [];

KF.Xkk_1 = zeros(n,1);
KF.Pkk_1 = zeros(n,n);

KF.Xk = zeros(n,1);
KF.Pk = zeros(n,n);

KF.Phikk_1 = zeros(n,n);

KF.Gk = zeros(n,k);
KF.Qk = zeros(k,k);

KF.Zk = zeros(m,1);

KF.Rk = zeros(m,m);

KF.Kk = zeros(n,m);

if mode == 1
    KF.Hk = [eye(3),zeros(3,3)];
end













