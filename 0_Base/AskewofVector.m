function M_v = AskewofVector(v)
% 将矢量转化为斜对成矩阵.
%
% Input:     v - 3x1 vector
% Output: v - corresponding 3x3 askew matrix, such that
%                     |  0   -v(3)  v(2) |
%           M_v = | v(3)  0    -v(1) |
%                     |-v(2)  v(1)  0    |
%


    M_v = [ 0,     -v(3),   v(2); 
          v(3),   0,     -v(1); 
         -v(2),   v(1),   0     ];