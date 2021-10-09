function Plot_IMU(IMU0,IMU1)
% 绘制原始的IMU数据 及 比对
%
%   （1）如果输入两组数据，第2组为参照数据(红色)..
%   （2）输入参数的格式为(按列) :
%           时间(单位s) 加计 xyz (g) 陀螺 xyz(rad/s)
%


if nargin == 1
%% 绘制加速度计信息 
    figure;
    set(gcf,'position',[250,250,1200,240]);
    subplot(1,3,1);
    plot(IMU0(:,1),IMU0(:,2));
    xlabel('\it t \rm / s');
    ylabel('\it \rm g');
    title('加计-x轴');
    
    subplot(1,3,2);
    plot(IMU0(:,1),IMU0(:,3));
    xlabel('\it t \rm / s');
    ylabel('\it \rm g');
    title('加计-y轴');
    
    subplot(1,3,3);
    plot(IMU0(:,1),IMU0(:,4));
    xlabel('\it t \rm / s');
    ylabel('\it \rm g');
    title('加计-z轴');

    
 %% 绘制陀螺信息 
    figure;
    set(gcf,'position',[250,250,1200,240]);
    subplot(1,3,1);
    plot(IMU0(:,1),IMU0(:,5));
    xlabel('\it t \rm / s');
    ylabel('\it \rm rad/s');
    title('陀螺-x轴');
    
    subplot(1,3,2);
    plot(IMU0(:,1),IMU0(:,6));
    xlabel('\it t \rm / s');
    ylabel('\it \rm rad/s');
    title('陀螺-y轴');
    
    subplot(1,3,3);
    plot(IMU0(:,1),IMU0(:,7));
    xlabel('\it t \rm / s');
    ylabel('\it \rm rad/s');
    title('陀螺-z轴');
    
    
end

if nargin == 2
%% 绘制加速度计信息 
    figure;
    set(gcf,'position',[250,250,1200,240]);
    subplot(1,3,1);
    plot(IMU0(:,1),IMU0(:,2),'b');
    hold on;
    plot(IMU1(:,1),IMU1(:,2),'r');   
    xlabel('\it t \rm / s');
    ylabel('\it \rm g');
    title('加计-x轴');
    
    subplot(1,3,2);
    plot(IMU0(:,1),IMU0(:,3),'b');
    hold on;
    plot(IMU1(:,1),IMU1(:,3),'r');       
    xlabel('\it t \rm / s');
    ylabel('\it \rm g');
    title('加计-y轴');
    
    subplot(1,3,3);
    plot(IMU0(:,1),IMU0(:,4),'b');
    hold on;
    plot(IMU1(:,1),IMU1(:,4),'r');       
    xlabel('\it t \rm / s');
    ylabel('\it \rm g');
    title('加计-z轴');

    
 %% 绘制陀螺信息 
    figure;
    set(gcf,'position',[250,250,1200,240]);
    subplot(1,3,1);
    plot(IMU0(:,1),IMU0(:,5),'b');
    hold on;
    plot(IMU1(:,1),IMU1(:,5),'r');       
    xlabel('\it t \rm / s');
    ylabel('\it \rm rad/s');
    title('陀螺-x轴');
    
    subplot(1,3,2);
    plot(IMU0(:,1),IMU0(:,6),'b');
     hold on;
    plot(IMU1(:,1),IMU1(:,6),'r');      
    xlabel('\it t \rm / s');
    ylabel('\it \rm rad/s');
    title('陀螺-y轴');
    
    subplot(1,3,3);
    plot(IMU0(:,1),IMU0(:,7),'b');
    hold on;
    plot(IMU1(:,1),IMU1(:,7),'r');       
    xlabel('\it t \rm / s');
    ylabel('\it \rm rad/s');
    title('陀螺-z轴');
end