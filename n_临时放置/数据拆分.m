%% 按照时间段，拆分数据
 % 待拆分数据 ：IMU
 
    % 设定拆分序号
    tmp_Start  = [1,        27500, 62360, 98060,131500];
    tmp_End   = [14000, 55000, 88100,121170,152100];
%     tmp_Start  = [77000,        177400, 280000, 367600,447000];
%     tmp_End   = [157000, 268500, 363200,442400,555500];
    
    zGyro1 = IMU(tmp_Start(1):tmp_End(1),7);
    zGyro2 = IMU(tmp_Start(2):tmp_End(2),7);
    zGyro3 = IMU(tmp_Start(3):tmp_End(3),7);
    zGyro4 = IMU(tmp_Start(4):tmp_End(4),7);
    zGyro5 = IMU(tmp_Start(5):tmp_End(5),7);    
    var(zGyro1) 
    var(zGyro2) 
    var(zGyro3) 
    var(zGyro4) 
    var(zGyro5)  
    
    yGyro1 = IMU(tmp_Start(1):tmp_End(1),6);
    yGyro2 = IMU(tmp_Start(2):tmp_End(2),6);
    yGyro3 = IMU(tmp_Start(3):tmp_End(3),6);
    yGyro4 = IMU(tmp_Start(4):tmp_End(4),6);
    yGyro5 = IMU(tmp_Start(5):tmp_End(5),6);
    
    var(yGyro1) 
    var(yGyro2) 
    var(yGyro3) 
    var(yGyro4) 
    var(yGyro5) 
    
    xGyro1 = IMU(tmp_Start(1):tmp_End(1),5);
    xGyro2 = IMU(tmp_Start(2):tmp_End(2),5);
    xGyro3 = IMU(tmp_Start(3):tmp_End(3),5);
    xGyro4 = IMU(tmp_Start(4):tmp_End(4),5);
    xGyro5 = IMU(tmp_Start(5):tmp_End(5),5);
    var(xGyro1) 
    var(xGyro2) 
    var(xGyro3) 
    var(xGyro4) 
    var(xGyro5) 