function w_en_n = EarthWen_n(lat,vn,Rmh,Rnh)
% 计算地球参数 n系相对e系转动角速度 投影 NED
%       导航坐标n系采用 北-东-地  NED
% Inputs:   lat 纬度，单位弧度
%           vn = [x;y;z] 载体相对地球速度在n系下的投影，单位弧度 m
% Output:   w_en_n = [x;y;z]      单位 弧度/s 
%


w_en_n = [vn(2,1)/Rnh;
                -vn(1,1)/Rmh;
                -vn(2,1)/Rnh*tan(lat)];
