function Cnb = AttChange_E2Mnb(Euler)
% 将输入的欧拉角转换为旋转矩阵 
% Inputs:   
%       欧拉角 单位 弧度  x 横滚 y俯仰 z航向  前右下  北东地
% Output:   旋转矩阵    Cnb  b系相对n系的旋转矩阵
%
s1 = sin(Euler(1)); c1 = cos(Euler(1));    % roll
s2 = sin(Euler(2)); c2 = cos(Euler(2));    % pitch
s3 = sin(Euler(3)); c3 = cos(Euler(3));    % yaw

Cnb = [c2*c3, c3*s2*s1-c1*s3, c1*c3*s2+s1*s3;
            c2*s3,  c1*c3+s1*s2*s3, c1*s2*s3-c3*s1;
            -s2,    c2*s1,  c2*c1];