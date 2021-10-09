function Plot_GyroBias(Bias0,Bias1)
% 绘制原始的IMU数据 及 比对
%
%   （1）如果输入两组数据，第2组为参照数据(红色)..
%   （2）输入参数的格式为(按列) :
%           时间(单位s) 陀螺 xyz 零偏 (rad/s)
%



if nargin == 1
%% 绘制陀螺零偏信息 
    figure;
    set(gcf,'position',[250,250,1200,240]);
    subplot(1,3,1);
    plot(Bias0(:,1),Bias0(:,2));
    xlabel('\it t \rm / s');
    ylabel('\it \rm rad/s');
    title('x轴陀螺零偏');
    
    subplot(1,3,2);
    plot(Bias0(:,1),Bias0(:,3));
    xlabel('\it t \rm / s');
    ylabel('\it \rm rad/s');
    title('y轴陀螺零偏');
    
    subplot(1,3,3);
    plot(Bias0(:,1),Bias0(:,4));
    xlabel('\it t \rm / s');
    ylabel('\it \rm rad/s');
    title('z轴陀螺零偏');    
    
end

if nargin == 2
%% 绘制加速度计信息 
    figure;
    set(gcf,'position',[250,250,1200,240]);
    subplot(1,3,1);
    plot(Bias0(:,1),Bias0(:,2),'b');
    hold on;
    plot(Bias1(:,1),Bias1(:,2),'r');   
    xlabel('\it t \rm / s');
    ylabel('\it \rm rad/s');
    title('x轴陀螺零偏');
    
    subplot(1,3,2);
    plot(Bias0(:,1),Bias0(:,3),'b');
    hold on;
    plot(Bias1(:,1),Bias1(:,3),'r');       
    xlabel('\it t \rm / s');
    ylabel('\it \rm rad/s');
    title('y轴陀螺零偏');
    
    subplot(1,3,3);
    plot(Bias0(:,1),Bias0(:,4),'b');
    hold on;
    plot(Bias1(:,1),Bias1(:,4),'r');       
    xlabel('\it t \rm / s');
    ylabel('\it \rm rad/s');
    title('z轴陀螺零偏');
 
end