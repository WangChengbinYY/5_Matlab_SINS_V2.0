clear;clc;

%  load('D:\IMUGPS2_200_152350.mat'); 
load('D:\MPU_R.mat')
IMU = IMU(:,2:8);
 
L = length(IMU);
 
Accx = IMU(:,2); Accy = IMU(:,3);  Accz = IMU(:,4); 
Gyrx = IMU(:,5); Gyry = IMU(:,6);  Gyrz = IMU(:,7);  

% 数据参数及存储
mWindow = 4; 

Record_GyrAngle1  = zeros(L,1);          % 陀螺实时角增量  有正有负
Record_GyrAngle4  = zeros(L,1);          % 陀螺实时角增量  有正有负
Record_GyrAngle8  = zeros(L,1);          % 陀螺实时角增量  有正有负

for i = 8:L
    Record_GyrAngle1(i,1) = abs(mean(Gyrx(i-1+1:i,1)));
    Record_GyrAngle4(i,1) = abs(mean(Gyrx(i-4+1:i,1)));
    Record_GyrAngle8(i,1) = abs(mean(Gyrx(i-8+1:i,1)));
end

newcolors = {'#F00','#0B0','#A0F','#00F','#50F','#F80'};
colororder(newcolors)
figure;
% plot(Gyrx);
hold on; plot(rad2deg(Record_GyrAngle1));
hold on; plot(rad2deg(Record_GyrAngle4));
hold on; plot(rad2deg(Record_GyrAngle8));
legend("1","4","8");
% legend("x","1","4","8");
title("陀螺x判断！")
% 
% Record_GyrAngle                    = zeros(L,1);          % 陀螺实时角增量  有正有负
% Record_GyrAngleWin             = zeros(L,1);           % 陀螺检测窗口内 的 平均角增量   正负平均一下
% Record_CheckMean              = zeros(L,1);           % 实时的检测均值
% Record_CheckThord              = zeros(L,1);           % 实时的阈值
% Record_State                          = zeros(L,1);           % 状态记录
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% CheckMean                            = 0;                           %实时的检测均值
% CheckThord                            = 0;                           %实时的检测阈值
% State                                        = 0;                           %当前时刻的状态
% StateChangeNum                   = 0;                           % 动态变为静态 的时刻
% 
% CheckTime                               = 0.2*200;                % 取0.2s的数据作为起始判定条件阈值
% 
% % 初始化
% Record_AccAmp(1:CheckTime,1)     = sqrt(Accx(1:CheckTime,1).^2+Accy(1:CheckTime,1).^2+Accz(1:CheckTime,1).^2);
% CheckMean = mean(Record_AccAmp(1:CheckTime,1));
% CheckThord = sqrt(var(Record_AccAmp(1:CheckTime,1)))*2;
% Record_AccAmpWinMean(1:CheckTime,1) = Record_AccAmp(1:CheckTime,1);
% Record_CheckMean(1:CheckTime,1) = CheckMean;
% Record_CheckThord(1:CheckTime,1) = CheckThord;
% Record_State(1:CheckTime,1) = 1;
% 
% for i = CheckTime : L
%     % 计算当前时刻的加速度计输出幅值
%         Record_AccAmp(i,1) = sqrt(Accx(i,1).^2+Accy(i,1).^2+Accz(i,1).^2);
%     % 当前时刻 窗口内加速度计幅值的均值
%         Record_AccAmpWinMean(i,1) = mean(Record_AccAmp(i-mWindow+1:i,1));        
%     if (Record_AccAmpWinMean(i,1) >= CheckMean-CheckThord) && (Record_AccAmpWinMean(i,1) <= CheckMean+CheckThord)
%         % 静止状态
%         if State == 0
%             % 由 动  变 静
%             State = 1;
%             StateChangeNum = i;
%         end
%             % 由 静 变 静
%             Record_State(i,1) = 1;
%             % 累积0.2s内的数据 更新判断阈值  当前时间窗口内的均值和方差
%         if (i-StateChangeNum >= CheckTime)
%             CheckMean = mean(Record_AccAmp(i-CheckTime+1:i,1));
%             Check3Rms = sqrt(var(Record_AccAmp(i-CheckTime+1:i,1)))*2;
%         end
%             Record_CheckMean(i,1) = CheckMean;
%             Record_CheckThord(i,1) = CheckThord;
%     else
%         % 运动状态
%         State = 0;
%         Record_State(i,1) = 0;
%         Record_CheckMean(i,1) = CheckMean;
%         Record_CheckThord(i,1) = CheckThord;
%     end    
% end
% 
% 
% % 绘图显示
% figure;
% plot((1:L),Record_AccAmp);
% % hold on;
% % plot((1:L),Record_State(:,1),'b');
% hold on;
% plot((1:L),Record_AccAmpWinMean,'r.-');
% hold on;
% plot((1:L),(Record_CheckMean+Record_CheckThord),'r');
% hold on;
% plot((1:L),(Record_CheckMean-Record_CheckThord),'r');
% 
% 
