function w_ie_n = EarthWie_n(G_CONST,lat)
% 计算地球参数 自转角速度 投影 NED
%       导航坐标n系采用 北-东-地  NED
% Inputs:     常量， 纬度，单位弧度
% Output:   w_ie_n = [x;y;z]      单位 弧度/s 
%

w_ie_n = [G_CONST.earth_wie*cos(lat); 0; -G_CONST.earth_wie*sin(lat)];