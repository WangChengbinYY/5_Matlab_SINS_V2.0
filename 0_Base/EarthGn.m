function g = EarthGn(G_Const,lat,h)
% 计算地球参数 重力加速度 使用纬度高程 
% Inputs:   纬度、高程，单位弧度 m
% Output:   g_n     单位 m/s2
 

g = G_Const.earth_g0*(1+5.27094e-3*sin(lat)^2+2.32718e-5*sin(lat)^4)-3.086e-6*h; % grs80
