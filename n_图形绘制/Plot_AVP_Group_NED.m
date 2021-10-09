function Plot_AVP_Group_NED(AVP0,AVP1)
% 将导航结果，姿态、速度、位置等分别绘制在一张图上
%
%   （1）如果输入两组数据，第2组为参照数据(红色)..
%   （2）输入参数的格式为(按列) :
%           时间(单位ms) 姿态(x横滚、y俯仰、z航向，单位弧度) ....
%           速度(x北、y东、z地) 
%           位置(纬度、经度，单位弧度，高程 单位米)
%           位置(以第一点为原点的，北向X，东向Y，高程)
%


if nargin == 1
%% 绘制姿态信息    
    figure;
    set(gcf,'position',[250,250,1200,240]);
    subplot(1,3,1);
    plot(AVP0(:,1),AVP0(:,2).*(180/pi));
    xlabel('\it t \rm / s');
    ylabel('\it \theta \rm / \circ');
    title('姿态-横滚');
    
    subplot(1,3,2);
    plot(AVP0(:,1),AVP0(:,3).*(180/pi));
    xlabel('\it t \rm / s');
    ylabel('\it \gamma \rm / \circ');
    title('姿态-俯仰');
    
    subplot(1,3,3);
    plot(AVP0(:,1),AVP0(:,4).*(180/pi));
    xlabel('\it t \rm / s');
    ylabel('\it \psi \rm / \circ');
    title('姿态-航向');
    
%% 绘制速度信息    
    figure;
    set(gcf,'position',[250,250,1200,240]);
    subplot(1,3,1);
    plot(AVP0(:,1),AVP0(:,5));
    xlabel('\it t \rm / s');
    ylabel('\it V_E \rm / m/s');
    title('速度-北向');

    subplot(1,3,2);
    plot(AVP0(:,1),AVP0(:,6));
    xlabel('\it t \rm / s');
    ylabel('\it V_N \rm / m/s');
    title('速度-东向');

    subplot(1,3,3);
    plot(AVP0(:,1),AVP0(:,7));
    xlabel('\it t \rm / s');
    ylabel('\it V_U \rm / m/s');
    title('速度-地向');   
  

%% 绘制位置信息  
    figure;
    set(gcf,'position',[250,250,1200,240]);
    subplot(1,3,1);
    plot(AVP0(:,1),AVP0(:,8).*(180/pi));
    xlabel('\it t \rm / s');
    ylabel('\it 纬度 \rm / \circ');
    title('位置-纬度');
    
    subplot(1,3,2);
    plot(AVP0(:,1),AVP0(:,9).*(180/pi));
    xlabel('\it t \rm / s');
    ylabel('\it 经度 \rm / \circ');
    title('位置-经度');
    
    subplot(1,3,3);
    plot(AVP0(:,1),AVP0(:,10));
    xlabel('\it t \rm / s');
    ylabel('\it 高程 \rm / m');
    title('位置-高程');    

%% 绘制轨迹信息    
    figure;
    subplot(1,3,1);
    plot(AVP0(:,1),(sqrt(AVP0(:,5).^2+AVP0(:,6).^2+AVP0(:,7).^2)));
    xlabel('\it t \rm / s');
    ylabel('\it V \rm / m/s');
    title('绝对速度');  
    
    
    %绘制以起点为出发点的行驶轨迹 经纬度
    subplot(1,3,2);
    plot(AVP0(1,9)*180/pi, AVP0(1,8)*180/pi, 'rp');     %在起始位置画一个 五角星
%     legend(sprintf('%.6f, %.6f / 度', AVP0(1,9)*180/pi,AVP0(1,8)*180/pi));
    hold on;    
    plot(AVP0(:,9).*(180/pi),AVP0(:,8).*(180/pi));
    xlabel('\it 经度 \rm / circ');
    ylabel('\it 纬度 \rm / circ');
    title('行驶路线(经纬度)');
    
    %绘制以起点为出发点的行驶轨迹 北东    
    subplot(1,3,3);    
    plot( AVP0(1,12),  AVP0(1,11), 'rp');     %在起始位置画一个 五角星
%     legend(sprintf('北%.3f,  东%.3f / 米', AVP0(1,11),  AVP0(1,12)));
    hold on;    
    plot(AVP0(:,12),AVP0(:,11));
    xlabel('\it 东向 \rm / m');
    ylabel('\it 北向 \rm / m');
    title('行驶路线');    
    
end

if nargin == 2
%% 绘制姿态信息    
    figure;
    set(gcf,'position',[250,250,1200,480]);
    subplot(1,3,1);
    plot(AVP0(:,1),AVP0(:,2).*(180/pi),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,2).*(180/pi),'r');    
    xlabel('\it t \rm / s');
    ylabel('\it \theta \rm / \circ');
    title('姿态-横滚');
    
    subplot(1,3,2);
    plot(AVP0(:,1),AVP0(:,3).*(180/pi),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,3).*(180/pi),'r');
    xlabel('\it t \rm / s');
    ylabel('\it \gamma \rm / \circ');
    title('姿态-俯仰');
    
    subplot(1,3,3);
    plot(AVP0(:,1),AVP0(:,4).*(180/pi),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,4).*(180/pi),'r');
    xlabel('\it t \rm / s');
    ylabel('\it \psi \rm / \circ');
    title('姿态-航向');
    
%% 绘制速度信息    
    figure;
    set(gcf,'position',[250,250,1200,480]);
    subplot(1,3,1);
    plot(AVP0(:,1),AVP0(:,5),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,5),'r');
    xlabel('\it t \rm / s');
    ylabel('\it V_E \rm / m/s');
    title('速度-北向');

    subplot(1,3,2);
    plot(AVP0(:,1),AVP0(:,6),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,6),'r');
    xlabel('\it t \rm / s');
    ylabel('\it V_N \rm / m/s');
    title('速度-东向');

    subplot(1,3,3);
    plot(AVP0(:,1),AVP0(:,7),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,7),'r');
    xlabel('\it t \rm / s');
    ylabel('\it V_U \rm / m/s');
    title('速度-地向');
    


%% 绘制位置信息  
    figure;
    set(gcf,'position',[250,250,1200,240]);
    subplot(1,3,1);
    plot(AVP0(:,1),AVP0(:,8).*(180/pi),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,8).*(180/pi),'r');    
    xlabel('\it t \rm / s');
    ylabel('\it 纬度 \rm / \circ');
    title('位置-纬度');
    
    subplot(1,3,2);
    plot(AVP0(:,1),AVP0(:,9).*(180/pi),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,9).*(180/pi),'r');
    xlabel('\it t \rm / s');
    ylabel('\it 经度 \rm / \circ');
    title('位置-经度');
    
    subplot(1,3,3);
    plot(AVP0(:,1),AVP0(:,10),'b');
    hold on;
    plot(AVP1(:,1),AVP1(:,10),'r');    
    xlabel('\it t \rm / s');
    ylabel('\it 高程 \rm / m');
    title('位置-高程');    

%% 绘制轨迹信息    
    figure;
    subplot(1,3,1);
    plot(AVP0(:,1),(sqrt(AVP0(:,5).^2+AVP0(:,6).^2+AVP0(:,7).^2)),'b');
    hold on;
    plot(AVP1(:,1),(sqrt(AVP1(:,5).^2+AVP1(:,6).^2+AVP1(:,7).^2)),'r');
    xlabel('\it t \rm / s');
    ylabel('\it V \rm / m/s');
    title('绝对速度');  
    
    
    %绘制以起点为出发点的行驶轨迹 经纬度
    subplot(1,3,2);
    plot(AVP0(1,9)*180/pi, AVP0(1,8)*180/pi, 'rp');     %在起始位置画一个 五角星
%     legend(sprintf('%.6f, %.6f / 度', AVP0(1,9)*180/pi,AVP0(1,8)*180/pi));
    hold on;    
    plot(AVP0(:,9).*(180/pi),AVP0(:,8).*(180/pi),'b');
    hold on;    
    plot(AVP1(:,9).*(180/pi),AVP1(:,8).*(180/pi),'r');
    xlabel('\it 经度 \rm / circ');
    ylabel('\it 纬度 \rm / circ');
    title('行驶路线(经纬度)');
    
    %绘制以起点为出发点的行驶轨迹 北东    
    subplot(1,3,3);    
    plot( AVP0(1,12),  AVP0(1,11), 'rp');     %在起始位置画一个 五角星
%     legend(sprintf('北%.3f,  东%.3f / 米', AVP0(1,11),  AVP0(1,12)));
    hold on;    
    plot(AVP0(:,12),AVP0(:,11),'b');
    hold on;    
    plot(AVP1(:,12),AVP1(:,11),'r');    
    xlabel('\it 东向 \rm / m');
    ylabel('\it 北向 \rm / m');
    title('行驶路线');    
    
end


