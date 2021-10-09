function Plot_KF_XkPk(n,XkPk0,XkPk1)
% 绘制KF滤波的中间状态
%
%   （1）如果输入两组数据，第2组为参照数据(红色)..
%   （2）输入参数的格式为(按列) :
%           时间(单位s) x1,x2....p1,p2......    pi为Pk(i,i)
%   （3）n 代表 是那种KF滤波

%% 模式1  基于旋转矩阵的 n' 假定的姿态静态滤波
if n == 1
    if nargin == 2
        % 姿态误差估计
        figure;
        set(gcf,'position',[250,250,1200,480]);    
        subplot(2,3,1);
        plot(XkPk0(:,1),XkPk0(:,2));
        xlabel('\it t \rm / s');
        title('Xk-x1');

        subplot(2,3,2);
        plot(XkPk0(:,1),XkPk0(:,3));
        xlabel('\it t \rm / s');
        title('Xk-x2');

        subplot(2,3,3);
        plot(XkPk0(:,1),XkPk0(:,4));
        xlabel('\it t \rm / s');
        title('Xk-x3');

        subplot(2,3,4);
        plot(XkPk0(:,1),XkPk0(:,8));
        xlabel('\it t \rm / s');
        title('Pk-P(1,1)');

        subplot(2,3,5);
        plot(XkPk0(:,1),XkPk0(:,9));
        xlabel('\it t \rm / s');
        title('Pk-P(2,2)');

        subplot(2,3,6);
        plot(XkPk0(:,1),XkPk0(:,10));
        xlabel('\it t \rm / s');
        title('Pk-P(3,3)');        
        
        % 零偏估计
        figure;
        set(gcf,'position',[250,250,1200,480]);    
        subplot(2,3,1);
        plot(XkPk0(:,1),XkPk0(:,5));
        xlabel('\it t \rm / s');
        title('Xk-x4');

        subplot(2,3,2);
        plot(XkPk0(:,1),XkPk0(:,6));
        xlabel('\it t \rm / s');
        title('Xk-x5');

        subplot(2,3,3);
        plot(XkPk0(:,1),XkPk0(:,7));
        xlabel('\it t \rm / s');
        title('Xk-x6');

        subplot(2,3,4);
        plot(XkPk0(:,1),XkPk0(:,11));
        xlabel('\it t \rm / s');
        title('Pk-P(4,4)');

        subplot(2,3,5);
        plot(XkPk0(:,1),XkPk0(:,12));
        xlabel('\it t \rm / s');
        title('Pk-P(5,5)');

        subplot(2,3,6);
        plot(XkPk0(:,1),XkPk0(:,13));
        xlabel('\it t \rm / s');
        title('Pk-P(6,6)');      
        
    end
    
    
    if nargin == 3
        % 姿态误差估计
        figure;
        set(gcf,'position',[250,250,1200,480]);    
        subplot(2,3,1);
        plot(XkPk0(:,1),XkPk0(:,2),'b');
        hold on; plot(XkPk1(:,1),XkPk1(:,2),'r');
        xlabel('\it t \rm / s');
        title('Xk-x1');

        subplot(2,3,2);
        plot(XkPk0(:,1),XkPk0(:,3),'b');
        hold on; plot(XkPk1(:,1),XkPk1(:,3),'r');        
        xlabel('\it t \rm / s');
        title('Xk-x2');

        subplot(2,3,3);
        plot(XkPk0(:,1),XkPk0(:,4),'b');
        hold on; plot(XkPk1(:,1),XkPk1(:,4),'r');                
        xlabel('\it t \rm / s');
        title('Xk-x3');

        subplot(2,3,4);
        plot(XkPk0(:,1),XkPk0(:,8),'b');
         hold on; plot(XkPk1(:,1),XkPk1(:,8),'r');               
        xlabel('\it t \rm / s');
        title('Pk-P(1,1)');

        subplot(2,3,5);
        plot(XkPk0(:,1),XkPk0(:,9),'b');
         hold on; plot(XkPk1(:,1),XkPk1(:,9),'r');               
        xlabel('\it t \rm / s');
        title('Pk-P(2,2)');

        subplot(2,3,6);
        plot(XkPk0(:,1),XkPk0(:,10),'b');
        hold on; plot(XkPk1(:,1),XkPk1(:,10),'r');               
        xlabel('\it t \rm / s');
        title('Pk-P(3,3)');        
        
        % 零偏估计
        figure;
        set(gcf,'position',[250,250,1200,480]);    
        subplot(2,3,1);
        plot(XkPk0(:,1),XkPk0(:,5),'b');
        hold on; plot(XkPk1(:,1),XkPk1(:,5),'r');                    
        xlabel('\it t \rm / s');
        title('Xk-x4');

        subplot(2,3,2);
        plot(XkPk0(:,1),XkPk0(:,6),'b');
        hold on; plot(XkPk1(:,1),XkPk1(:,6),'r');    
        xlabel('\it t \rm / s');
        title('Xk-x5');

        subplot(2,3,3);
        plot(XkPk0(:,1),XkPk0(:,7),'b');
        hold on; plot(XkPk1(:,1),XkPk1(:,7),'r');    
        xlabel('\it t \rm / s');
        title('Xk-x6');

        subplot(2,3,4);
        plot(XkPk0(:,1),XkPk0(:,11),'b');
        hold on; plot(XkPk1(:,1),XkPk1(:,11),'r');    
        xlabel('\it t \rm / s');
        title('Pk-P(4,4)');

        subplot(2,3,5);
        plot(XkPk0(:,1),XkPk0(:,12),'b');
        hold on; plot(XkPk1(:,1),XkPk1(:,12),'r');    
        xlabel('\it t \rm / s');
        title('Pk-P(5,5)');

        subplot(2,3,6);
        plot(XkPk0(:,1),XkPk0(:,13),'b');
        hold on; plot(XkPk1(:,1),XkPk1(:,13),'r');    
        xlabel('\it t \rm / s');
        title('Pk-P(6,6)');                   
    end    
end
