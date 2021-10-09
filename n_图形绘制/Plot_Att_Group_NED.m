function Plot_Att_Group_NED(Att0,Att1)
% 将姿态欧拉角绘制在一张图上
%
%   （1）如果输入两组数据，第2组为参照数据(红色)..
%   （2）输入参数的格式为(按列) :
%           时间(单位ms) 姿态(x横滚、y俯仰、z航向，单位弧度)
%



if nargin == 1
%% 绘制姿态信息    
    figure;
    set(gcf,'position',[250,250,1200,240]);
    subplot(1,3,1);
    plot(Att0(:,1),Att0(:,2).*(180/pi));
    xlabel('\it t \rm / s');
    ylabel('\it \theta \rm / \circ');
    title('姿态-横滚');
    
    subplot(1,3,2);
    plot(Att0(:,1),Att0(:,3).*(180/pi));
    xlabel('\it t \rm / s');
    ylabel('\it \gamma \rm / \circ');
    title('姿态-俯仰');
    
    subplot(1,3,3);
    plot(Att0(:,1),Att0(:,4).*(180/pi));
    xlabel('\it t \rm / s');
    ylabel('\it \psi \rm / \circ');
    title('姿态-航向');

end

if nargin == 2
%% 绘制姿态信息    
    figure;
    set(gcf,'position',[250,250,1200,480]);
    subplot(1,3,1);
    plot(Att0(:,1),Att0(:,2).*(180/pi),'b');
    hold on;
    plot(Att1(:,1),Att1(:,2).*(180/pi),'r');    
    xlabel('\it t \rm / s');
    ylabel('\it \theta \rm / \circ');
    title('姿态-横滚');
    
    subplot(1,3,2);
    plot(Att0(:,1),Att0(:,3).*(180/pi),'b');
    hold on;
    plot(Att1(:,1),Att1(:,3).*(180/pi),'r');
    xlabel('\it t \rm / s');
    ylabel('\it \gamma \rm / \circ');
    title('姿态-俯仰');
    
    subplot(1,3,3);
    plot(Att0(:,1),Att0(:,4).*(180/pi),'b');
    hold on;
    plot(Att1(:,1),Att1(:,4).*(180/pi),'r');
    xlabel('\it t \rm / s');
    ylabel('\it \psi \rm / \circ');
    title('姿态-航向');
end
