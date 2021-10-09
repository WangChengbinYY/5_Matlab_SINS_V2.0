function ZeroDetection_AG(str)
%% 零速检测试验

% 读取数据
    load(str);
    L = length(IMU);

% 参数设置    
    Check_TimeWindow = 4;               % 检测窗口长度   
    Check_Acc_Mean = 0;                  % 加速度计输出检测阈值 均值 和 rms上下线
    Check_Acc_Rms = 0;    
    Check_Gyr_X = 0;                        % 陀螺输出检测，两种方式，x轴陀螺输出角速率 or 旋转矢量角增量
%     Check_Gyr_Angle = 0;   
    Check_State_pre = 0;                           % 前一时刻状态
    Check_State = 0;                                % 当前时刻状态
    Check_State_A = 0;                          % 加计检测状态
    Check_State_G = 0;                          % 陀螺检测状态
    
% 数据存储记录
    % 加计相关的记录
    Record_Acc_Amp = zeros(L,1);    % 每一时刻的加计幅值
    Record_Acc_Mean = zeros(L,1);   % 每个窗口内加计幅值的均值
    Record_Acc_Rms = zeros(L,1);    % 每个检测窗口内 加速度计幅值的 均方差
    Record_Check_Acc_Mean = zeros(L,1);     % 加计检测 均值阈值
    Record_Check_Acc_Rms = zeros(L,1);          %加计检测 方差阈值
    % 陀螺
    Record_Gyr_XMean = zeros(L,1);              % 每个检测窗口内ｘ轴陀螺的平均角速率
%     Record_Gyr_AnglMean = zeros(L,1);        % 每个检测窗口内3轴陀螺的平均角增量
    Record_Check_Gyr_X = zeros(L,1);            % x轴陀螺检测阈值
    % 状态
    Record_State = zeros(L,1);
    % 每一步的时间长度
    Record_StaticTime = zeros(1,3);          % 每一步静止状态  起始  结束 总时间(采样个数)
    StaticTime_Num = 0;                           % 当前静止步数记录个数  第一个运动状态开始计算，以防止长时间静止踩在地面上
    
% 检测参数初始化
    % 初始化时间 10s
    tmp_Start = 10*200;
    % 加计检测参数初始化
    tmp_AMP = sqrt(IMU(1:tmp_Start,2).^2+IMU(1:tmp_Start,3).^2+IMU(1:tmp_Start,4).^2);
    Check_Acc_Mean = mean(tmp_AMP);     % 包含加计的零偏
    Check_Acc_Rms = sqrt(var(tmp_AMP))*3;    % 初始 3倍rms 
    % 陀螺检测参数初始化
%     Check_Gyr_X = abs(mean(IMU(1:tmp_Start,5)));     %约 3 deg/s
    Check_Gyr_X = 1;      % 57 deg/s
%     tmp_Gyr = mean(IMU(1:tmp_Start,5:7));
%     Check_Gyr_Angle = norm(tmp_Gyr);


    
% 按检测窗口循环检测
    % 简单起见，初始状态为静止
    Check_State = 1;
    Check_State_pre = 1;
    StaticTime_Num = 1;
    tmp_Static_Start = 1;   %静止状态的起始时刻
    tmp_Static_End = 0;     %静止状态的结束时刻
    Record_StaticTime(1,1) = 1;       
    tmp_IsStatic = 0;           % 是否是真实的静止阶段 用于标识 静止阶段起终的记录
    tmp_Change = 0;         % 1 代表 运动—>静止   -1 代表 静止—>运动
    tmp_StaticLength = 100;     % 一步中间静止的采样时间长度
    tmp_StaticLengthXi = 0.8;
        
    for i = 1:L
        
        %每一时刻的加计幅值
        Record_Acc_Amp(i,1) = norm(IMU(i,2:4));
        
        if mod(i,Check_TimeWindow) == 0
            
        % 进入检测
        
            %获取当前检测窗口内的加计和陀螺数据
                tmp_Acc = IMU(i-Check_TimeWindow+1:i,2:4);
                tmp_Gyr = IMU(i-Check_TimeWindow+1:i,5:7);
                
            %计算检测信息
                Check_State_A = ZeroDetect_Acc(tmp_Acc,Check_TimeWindow,Check_Acc_Mean,Check_Acc_Rms);
                Check_State_G = ZeroDetect_Gyr_X(tmp_Gyr,Check_TimeWindow,Check_Gyr_X);     
                
            %开始检测判断
            
%--------------- 前一时刻为静态-----------------
                if Check_State_pre == 1
                    
      %------加计静止
                    if Check_State_A == 1   
                        
              %----陀螺静止
                        if Check_State_G == 1
                            Check_State = 1;
                        else
                            
              %----陀螺运动
                            if StaticTime_Num <= 1
                                Check_State = 1;      % 第一步的静止判断，仅依靠加计，不考虑陀螺
                            else
                                % 当前静止持续的时间长度
                                tmp = i - tmp_Static_Start;
                                % 静止步态时间不够  静止阶段过程中，陀螺噪声造成
                                if tmp < tmp_StaticLength*tmp_StaticLengthXi
                                    % 准静态
                                    Check_State = 0;
                                else
                                    % 状态改变*******************************************静—>动
                                    Check_State = -1;
                                    tmp_Change = -1;   
                                    tmp_Static_End = i;
                                                                     
                                end
                            end                              
                        end 
                        
                    end
                    
      %------加计准静止
                    if Check_State_A == 0
              %----陀螺静止
                        if Check_State_G == 1
                            % 加计噪声比较大，状态为 准静止
                            Check_State = 0;
              %----陀螺运动
                        else
                            % 状态改变************************************************静—>动
                            Check_State = -1;
                            tmp_Change = -1; 
                            tmp_Static_End = i;
                        end
                    end
                    
      %------加计运动
                    if Check_State_A == -1
                        % 状态改变*************************************************** 静—>动
                        Check_State = -1;
                        tmp_Change = -1; 
                        tmp_Static_End = i;                       
                    end                     
                    
                end            
                
%---------前一时刻为准静态-----------------
                if Check_State_pre == 0
                    
      %------加计静止
                    if Check_State_A == 1
              %----陀螺静止
                        if Check_State_G == 1
                            Check_State = 1;
                        else
              %----陀螺运动
                            if StaticTime_Num <= 1
                                Check_State = 1;      % 第一步的静止判断，仅依靠加计，不考虑陀螺
                            else
                                % 当前静止持续的时间长度
                                tmp = i - tmp_Static_Start;
                                % 静止步态时间不够  静止阶段过程中，陀螺噪声造成
                                if tmp < tmp_StaticLength*tmp_StaticLengthXi
                                    % 准静态
                                    Check_State = 0;
                                else
                                    % 状态改变*******************************************静—>动
                                    Check_State = -1;
                                    tmp_Change = -1;   
                                    tmp_Static_End = i;                                                                     
                                end    
                            end 
                        end   
                    end                    
                    
      %------加计准静止
                    if Check_State_A == 0
              %----陀螺静止
                        if Check_State_G == 1
                            % 加计噪声比较大，状态为 准静止
                            Check_State = 0;
              %----陀螺运动
                        else
                            % 状态改变************************************************静—>动
                            Check_State = -1;
                            tmp_Change = -1; 
                            tmp_Static_End = i;
                        end
                    end
                    
      %------加计运动
                    if Check_State_A == -1
                        % 状态改变*************************************************** 静—>动
                        Check_State = -1;
                        tmp_Change = -1; 
                        tmp_Static_End = i;                       
                    end          
                    
                end               
            
%---------------前一时刻为动态------------------
                if Check_State_pre == -1
                    
      %------加计静止
                    if Check_State_A == 1
              %----陀螺静止
                        if Check_State_G == 1
                            % 状态改变***************************************************动—>静
                            Check_State = 1;                            
                            tmp_Change = 1;
                            tmp_Static_Start = i;
                        end
                    end
                    
      %------加计准静止
                    if Check_State_A == 0
                        % 状态不变
                        Check_State = -1;
                    end
                    
      %------加计运动
                    if Check_State_A == 0
                        % 状态不变
                        Check_State = -1;
                    end                    
                end

                  
%-------记录加速度计和陀螺输出信息特征
                Record_State(i-Check_TimeWindow+1:i,1) = Check_State;
                Record_Acc_Mean(i-Check_TimeWindow+1:i,1) = mean(Record_Acc_Amp(i-Check_TimeWindow+1:i,1));
                Record_Acc_Rms(i-Check_TimeWindow+1:i,1) = sqrt(var(Record_Acc_Amp(i-Check_TimeWindow+1:i,1)));
                Record_Gyr_XMean(i-Check_TimeWindow+1:i,1) = mean(IMU(i-Check_TimeWindow+1:i,5));
%-------记录检测阈值信息                
                Record_Check_Acc_Mean(i-Check_TimeWindow+1:i,1) = Check_Acc_Mean;
                Record_Check_Acc_Rms(i-Check_TimeWindow+1:i,1) = Check_Acc_Rms;
                Record_Check_Gyr_X(i-Check_TimeWindow+1:i,1) = Check_Gyr_X;
                
                Check_State_pre = Check_State;
            
%-------更新检测阈值
                % 由运动 变为 静止
                if tmp_Change == 1 
                    tmp_Change = 0;
                    %记录静止阶段的起始信息
                    Record_StaticTime(StaticTime_Num,1) = tmp_Static_Start;                     
                end

                % 静止 变为 运动
                if tmp_Change == -1 
                    tmp_Change = 0;
                    % 判断当前的静止阶段是否是真实的静止阶段
                        m = tmp_Static_End - tmp_Static_Start + 1;
                        if m >= tmp_StaticLength*tmp_StaticLengthXi/2
                            %记录静止阶段的终止信息
                            Record_StaticTime(StaticTime_Num,2) = tmp_Static_End;             
                            Record_StaticTime(StaticTime_Num,3) = tmp_Static_End-tmp_Static_Start+1;     
                            StaticTime_Num = StaticTime_Num + 1;     
 
                                % 长时间静止时，不进行阈值特征的更新 否则会造成阈值特别小
                                if m < tmp_StaticLength*3
                                    % 去掉头尾10% 的数据 取中间稳定的数据 
                                    h = 0.1;
                                    tmp_m = fix(m*h);
                                    tmp_start = tmp_Static_Start + tmp_m;
                                    tmp_end = tmp_Static_End-tmp_m;

                                    % 加计阈值更新
                                    tmp_Acc = IMU(tmp_start:tmp_end,2:4);
                                    tmp_Amp = sqrt(tmp_Acc(:,1).^2+tmp_Acc(:,2).^2+tmp_Acc(:,3).^2);
                                    Check_Acc_Mean = mean(tmp_Amp);
                                    Check_Acc_Rms = sqrt(var(tmp_Amp));

                                    %陀螺阈值更新
                                    tmp_Gyrx = IMU(tmp_start:tmp_end,5);
                                    tmp_Gyrx_mean = mean(tmp_Gyrx);
                                    tmp_Gyrx_rms = sqrt(var(tmp_Gyrx));
                                    if tmp_Gyrx_mean < 0
                                        Check_Gyr_X = abs(tmp_Gyrx_mean-tmp_Gyrx_rms*3);
                                    else
                                        Check_Gyr_X = abs(tmp_Gyrx_mean+tmp_Gyrx_rms*3);
                                    end    

                                	tmp_StaticLength = m;         
                                end
                            
                        end
                        if m < 0
                            disp("Static End Start is Wrong!");
                        end        
                   
                end
        end                    
    end                
            


    % 结果绘制
    figure;
    plot((1:L),Record_Acc_Amp,'m');
    hold on; plot((1:L),IMU(:,5),'b');
    hold on; plot((1:L),Record_State,'r');

    figure;
    plot((1:L),Record_Acc_Mean,'LineWidth',2,'Color','m');
    hold on; plot((1:L),(Record_Check_Acc_Mean+Record_Check_Acc_Rms.*3),'m');
    hold on; plot((1:L),(Record_Check_Acc_Mean-Record_Check_Acc_Rms.*3),'m');
    title("加计幅值判断");
    
    figure;
    plot((1:L),abs(Record_Gyr_XMean),'LineWidth',2,'Color','b');
    hold on; plot((1:L),Record_Check_Gyr_X,'b');
    title("X轴陀螺判断");
    
    i = 1;
    
    function r = ZeroDetect_Acc(acc,timewindow,checkmean,checkrms)
% acc  x y z 加速度计输入 单位g
% 判定范围  <3rms  r=1; 3rms~5rms r=0; >5rms r=-1;

% 按照 timewindow 长度进行加速度计输出检测
    % 1.计算平均幅值
    tmp = sqrt(acc(1:timewindow,1).^2+acc(1:timewindow,2).^2+acc(1:timewindow,3).^2);
    tmp_mean = mean(tmp);
    
    tmp_error = abs(tmp_mean-checkmean);
    
    if tmp_error <= 3*checkrms
        r = 1;
    else
        if tmp_error <= 5*checkrms
            r = 0;
        else
            r = -1;
        end
    end
    
    
function r = ZeroDetect_Gyr_X(wx,timewindow,checkmin)
% wx x轴陀螺角速度  
% 判定范围  <=checkmin  r=1; >checkmin r=-1; 1 静  -1动

% 按照 timewindow 长度进行x轴陀螺角速度判断
    % 1.计算平均角速度
    tmp_mean = mean(wx(1:timewindow,1));
    if (abs(tmp_mean) <= checkmin)
        r = 1;
    else
        r = -1;
    end


function r = ZeroDetect_Gyr_Angle(gyr,timewindow,checkmin)
% gyr 3轴陀螺角速度 x y z   
% checkmin 最小角增量  弧度/s 起始也就是平均角速度
% 判定范围  <=checkmin  r=1; >checkmin r=-1; 1 静  -1 动
% 考虑3个轴的陀螺零偏和地球自转角速度的情况下，timewindow检测窗口内，3轴陀螺输出
% 累计角度超过一定限定checkmin后的判定

% 按照 timewindow 长度进行3轴陀螺角增量的判断
% **** 待测试小角度进行 双子样姿态更新 和 小角度直接叠加 误差能有多少！！！！ 
%  此处先按照直接累计 后 求模 实现

    % 1.计算时间窗口内3轴平均角速度
    tmp_mean = mean(gyr(1:timewindow,1:3));
    tmp_angle = norm(tmp_mean);
    if (tmp_angle <= checkmin)
        r = 1;
    else
        r = -1;
    end


