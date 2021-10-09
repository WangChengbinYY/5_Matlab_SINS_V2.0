function Const = InitConst()
%初始化 全局变量及常值

%% 地球常值参数    
    Const.earth_wie   = 7.2921151467e-5;              %地球自转角速度 标量 单位：弧度/s
    Const.earth_f     = 0.003352813177897;
    Const.earth_Re    = 6378137;                      %单位：m
    Const.earth_e     = 0.081819221455524;    
    Const.earth_g0    = 9.7803267714;                 %单位：m/s2   
       
    
    
%% 单位换算相关
    Const.PI         = 3.141592653589793;               %后面使用为pi，C语言中宏定义    
    Const.D2R        = Const.PI/180.0;                   %度转弧度
    Const.R2D        = 180.0/Const.PI;                   %弧度转度

    Const.g0         = 9.7803267714;                    %单位：m/s2
    Const.mg         = 1.0e-3*Const.g0;                  %单位：m/s2
    Const.ug         = 1.0e-6*Const.g0;                  %单位：m/s2
    Const.mGal       = 1.0e-3*0.01;                     %单位：m/s2
    

        