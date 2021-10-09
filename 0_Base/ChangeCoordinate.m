function IMU_new = ChangeCoordinate(IMU,direction,n)
% 变换IMU原始数据的坐标方向
% Inputs:   IMU 数据，direction变换方向，1代表'ENU' 变为NED；
%               n 为输入数据种类选择
%                   n = 1:  
%                       时间 加计xyz 陀螺xyz 其它
%                   n = 2:
%                       时间 时间 加计xyz 陀螺xyz 其它
% Output:  IMU 变换后的数据

% 右脚： ENU  变为  NED 
%       xyz —> y x -z

IMU_new = IMU;

if direction == 1 
    if n == 1
        IMU_new(:,2) = IMU(:,3);
        IMU_new(:,3) = IMU(:,2);
        IMU_new(:,4) = -IMU(:,4);
        IMU_new(:,5) = IMU(:,6);
        IMU_new(:,6) = IMU(:,5);
        IMU_new(:,7) = -IMU(:,7);        
    end    
    if n == 2
        IMU_new(:,3) = IMU(:,4);
        IMU_new(:,4) = IMU(:,3);
        IMU_new(:,5) = -IMU(:,5);
        IMU_new(:,6) = IMU(:,7);
        IMU_new(:,7) = IMU(:,6);
        IMU_new(:,8) = -IMU(:,8);        
    end
end


% 左脚：




% 数据转换失败
