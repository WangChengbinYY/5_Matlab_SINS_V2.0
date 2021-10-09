function Rmh = EarthRmh(G_CONST,lat,h)
% 计算地球参数 子午圈半径 使用纬度高程 
% Inputs:   常量， 纬度、高程，单位弧度 m
% Output:   Rmh     单位 m
%

Rmh = G_CONST.earth_Re*(1-G_CONST.earth_e^2)/(1-G_CONST.earth_e^2*sin(lat)^2)^1.5 + h;