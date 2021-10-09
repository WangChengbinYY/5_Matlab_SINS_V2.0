%% 加速度计姿态估计 误差分析 仿真



%% 俯仰角误差仿真
clear;clc;
L = 50;           % 仿真的数据长度
e_max = 20 / 1000;      % mg  加计误差   

e = 0:(e_max/L):e_max-(e_max/L);  % 加速度计待增加的误差
e = e';
Eulerd = [25,15,8];                           % 理论姿态



Eulerd_Right = zeros(L,3);            % Z  Y  X
Eulerd_Right(:,1) = 25;
Eulerd_Right(:,3) = 8;
Eulerd_Right(:,2) =(0:88/L:88-88/L)';

g_n = [0,0,1];
m_b = [1,0,0];

X = zeros(L,L); Y = zeros(L,L); Z = zeros(L,L);  Z_Model = zeros(L,L); 

for i = 1:L
    X(:,i) = Eulerd_Right(i,2);
    for j = 1:L
        Y(j,:) = e(j);
        
        % 计算理论的g_b
        tp_Eulerd = Eulerd_Right(i,:);   %  Z Y X
        Q = quaternion(tp_Eulerd,'eulerd','ZYX','frame');
        g_b_right = rotateframe(Q, g_n);
        
        % g_b叠加误差
        g_b_error = g_b_right + e(j);
        
        % 计算有误差的姿态
        tp_Q_error = ecompass(g_b_error,m_b);
        tp_Eulerd_error = eulerd(tp_Q_error,'ZYX','frame');
        Z(j,i) = tp_Eulerd_error(2) - tp_Eulerd(2);
        
        % 计算模型姿态误差
        Z_Model(j,i) = rad2deg( ((norm(g_b_error)-1)*g_b_right(1)-e(j)) / norm(g_b_error) / cos(deg2rad(tp_Eulerd(2))) );              
        
    end    
end

mesh(X,Y,Z)
xlabel('\it e_x,e_y,e_z \rm / g')
ylabel('\it \theta \rm / \circ')
zlabel('\it \Delta\theta \rm / \circ')
caz = 251.4;   cel = 13.82;
view(caz,cel);
ax = gca;
ax.FontSize = 12;

mesh(X,Y,Z_Model)
xlabel('\it e_x,e_y,e_z \rm / g')
ylabel('\it \theta \rm / \circ')
zlabel('\it \Delta\theta \rm / \circ')
caz = 251.4;   cel = 13.82;
view(caz,cel);
ax = gca;
ax.FontSize = 12;


%% 横滚角误差仿真
clear;clc;
L = 50;           % 仿真的数据长度
e_max = 20 / 1000;      % mg  加计误差   

e = 0:(e_max/L):e_max-(e_max/L);  % 加速度计待增加的误差
e = e';
Eulerd = [25,15,8];                           % 理论姿态
Eulerd_Right = zeros(L,3);            % Z  Y  X
Eulerd_Right(:,1) = 25;
Eulerd_Right(:,2) = 15;
Eulerd_Right(:,3) =(0:178/L:178-178/L)';

g_n = [0,0,1];
m_b = [1,0,0];

X = zeros(L,L); Y = zeros(L,L); Z = zeros(L,L);  Z_Model = zeros(L,L); 
for i = 1:L
    X(:,i) = Eulerd_Right(i,3);
    for j = 1:L
        Y(j,:) = e(j);
        
        % 计算理论的g_b
        tp_Eulerd = Eulerd_Right(i,:);   %  Z Y X
        Q = quaternion(tp_Eulerd,'eulerd','ZYX','frame');
        g_b_right = rotateframe(Q, g_n);
        
        % g_b叠加误差
        g_b_error = g_b_right + e(j);
        
        % 计算有误差的横滚角
        tp_Q_error = ecompass(g_b_error,m_b);
        tp_Eulerd_error = eulerd(tp_Q_error,'ZYX','frame');
        Z(j,i) = tp_Eulerd_error(3) - tp_Eulerd(3);
        
        % 计算模型横滚角误差
%         Z_Model(j,i) = rad2deg( (e(j)*g_b_right(3)-e(j)*g_b_right(2)) / g_b_right(3) / (g_b_right(3)+e(j)) / (1+tan(deg2rad(tp_Eulerd(3)))^2));     
        Z_Model(j,i) = rad2deg(  (e(j)*g_b_right(3)^2-e(j)*g_b_right(2)*g_b_right(3) ) / (g_b_right(3)+e(j)) / cos(deg2rad(tp_Eulerd(2)))^2 );     
        
    end    
end
mesh(X,Y,Z)
xlabel('\it e_x,e_y,e_z \rm / g')
ylabel('\it \phi \rm / \circ')
zlabel('\it \Delta\phi \rm / \circ')
caz = -132.6;   cel = 22.63;
view(caz,cel);
ax = gca;
ax.FontSize = 12;

mesh(X,Y,Z_Model)
xlabel('\it e_x,e_y,e_z \rm / g')
ylabel('\it \phi \rm / \circ')
zlabel('\it \Delta\phi \rm / \circ')
caz = -132.6;   cel = 22.63;
view(caz,cel);
ax = gca;
ax.FontSize = 12;



%%  方差验证 横滚 俯仰
clear;clc;
L = 600;           % 仿真的数据长度
Eulerd = [45,25,13];                           % 理论姿态  Z Y X
Q = quaternion(Eulerd,'eulerd','ZYX','frame');
g_n = [0,0,1];
g_b = rotateframe(Q, g_n);
m_b = [1,0,0];

% 加计噪声  三个轴的
Sigmax = 0.005^2; Sigmay = 0.008^2; Sigmaz = 0.009^2; 
ex = wgn(L,1,Sigmax,'linear');
ey = wgn(L,1,Sigmay,'linear');
ez = wgn(L,1,Sigmaz,'linear');

% 姿态误差
Euler_erro = zeros(L,3);
g_b_e_draw = zeros(L,3);
Eulerd_draw = zeros(L,3);
for i = 1:L
    g_b_e = g_b + [ex(i),ey(i),ez(i)];
    g_b_e_draw(i,:) = g_b_e;
    orientation = ecompass(g_b_e,m_b);
    Eulerd_tp = eulerd(orientation,'ZYX','frame');
    Eulerd_draw(i,:) = Eulerd_tp;
    Euler_erro(i,:) = Eulerd_tp - Eulerd;
end

%俯仰角误差 理论方差
    Erro_Y = deg2rad(Euler_erro(:,2));   
    rad2deg(sqrt(var(Erro_Y)))
% 俯仰角误差 模型方差
    fx = g_b(1); fy = g_b(2); fz = g_b(3); 
    Erro_Y_model = ((1-fx^2)^2*Sigmax + fx^2*fy^2*Sigmay+fx^2*fz^2*Sigmaz)/cos(deg2rad(Eulerd(2)))^2;
    rad2deg(sqrt(Erro_Y_model))

% 横滚角误差  理论方差
    Erro_X = deg2rad(Euler_erro(:,3));   
    rad2deg(sqrt(var(Erro_X)))
% 横滚角误差 模型方差
    fx = g_b(1); fy = g_b(2); fz = g_b(3); 
    Erro_X_model = (fz/(fz^2+fy^2))^2*Sigmay + (fy/fz^2/(fz^2+fy^2))^2*Sigmaz;
    rad2deg(sqrt(Erro_X_model))

% 图形绘制
    time = (1:L)./200;
    figure;
    subplot(3,1,1);
    plot(time,g_b_e_draw(:,1));
    hold on;
    plot(time,ones(L).*g_b(1),'r');
    ylabel('\it Acc x \rm / g');
    legend("noised value","theoretical value")    
    subplot(3,1,2);
    plot(time,g_b_e_draw(:,2));
    hold on;
    plot(time,ones(L).*g_b(2),'r');
    ylabel('\it Acc y \rm / g');
    subplot(3,1,3);
    plot(time,g_b_e_draw(:,3));
    hold on;
    plot(time,ones(L).*g_b(3),'r');    
    ylabel('\it Acc z \rm / g');
    xlabel('\it time \rm / s');
    
    time = (1:L)./200;    
    figure;
    subplot(2,1,1);
    plot(time,Eulerd_draw(:,3));
    hold on;
    plot(time,ones(L).*Eulerd(3),'r');
    ylabel('\it \phi \rm / \circ')
    legend("noised value","theoretical value")    
    subplot(2,1,2);
    plot(time,Eulerd_draw(:,2));
    hold on;
    plot(time,ones(L).*Eulerd(2),'r');
    ylabel('\it \theta \rm / \circ')
    xlabel('\it time \rm / s');   
    
    
    
%%  方差随机50组 统计
clear;clc;
L = 600;           % 仿真的数据长度
N = 50;             % 仿真的数据组数
Eulerd(:,1) = randi([0,180],1,N);  % 航向 Z
Eulerd(:,2) = randi([0,50],1,N);  % 俯仰 Y
Eulerd(:,3) = randi([0,50],1,N);  % 横滚 X

Sigmax = randi([0,10^2],1,N)./1000^2;
Sigmay = randi([0,10^2],1,N)./1000^2;
Sigmaz = randi([0,10^2],1,N)./1000^2;

Eulerd_var_right = zeros(N,2);
Eulerd_var_erro = zeros(N,2);

g_n = [0,0,1];
m_b = [1,0,0];

for j = 1:N
    % 计算g_n的投影
        Q = quaternion(Eulerd(j,:),'eulerd','ZYX','frame');
        g_b = rotateframe(Q, g_n);
    
    % 产生加计噪声
        ex = wgn(L,1,Sigmax(j),'linear');
        ey = wgn(L,1,Sigmay(j),'linear');
        ez = wgn(L,1,Sigmaz(j),'linear');    
        
     % 姿态误差存储
        Euler_erro = zeros(L,3);
        
     % 计算有误差的姿态
        for i = 1:L
            % 加计叠加噪声
                g_b_e = g_b + [ex(i),ey(i),ez(i)];
                orientation = ecompass(g_b_e,m_b);
                Eulerd_tp = eulerd(orientation,'ZYX','frame');
                Euler_erro(i,:) = Eulerd_tp - Eulerd(j,:);                               
        end
        
    % 统计分析 记录
        % 俯仰角理论均方差
        Erro_Y = deg2rad(Euler_erro(:,2));           
        Eulerd_var_right(j,1) = rad2deg(sqrt(var(Erro_Y)));
        
        % 俯仰角误差 模型方差
        fx = g_b(1); fy = g_b(2); fz = g_b(3); 
        Erro_Y_model = ((1-fx^2)^2*Sigmax(j) + fx^2*fy^2*Sigmay(j)+fx^2*fz^2*Sigmaz(j))/cos(deg2rad(Eulerd(j,2)))^2;
        Eulerd_var_erro(j,1) = rad2deg(sqrt(Erro_Y_model));

        % 横滚角 理论均方差
        Erro_X = deg2rad(Euler_erro(:,3));   
        Eulerd_var_right(j,2) = rad2deg(sqrt(var(Erro_X)));

        % 横滚角误差 模型方差
        fx = g_b(1); fy = g_b(2); fz = g_b(3); 
        Erro_X_model = (fz/(fz^2+fy^2))^2*Sigmay(j) + (fy/fz^2/(fz^2+fy^2))^2*Sigmaz(j);
        Eulerd_var_erro(j,2) = rad2deg(sqrt(Erro_X_model));       
    
end

% 绘制比对图片
    figure;
    plot((1:N),Eulerd_var_erro(:,1) );
    hold on;
    plot((1:N),Eulerd_var_right(:,1),'r');
    ylabel('\it \sigma_\phi \rm / \circ');
    xlabel('\it samples');   
    legend("model value","theoretical value");

    figure;
    plot((1:N),Eulerd_var_erro(:,2) );
    hold on;
    plot((1:N),Eulerd_var_right(:,2),'r');
    ylabel('\it \sigma_\theta \rm / \circ');
    xlabel('\it samples');   
    legend("model value","theoretical value");

