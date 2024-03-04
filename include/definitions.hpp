Position current_position = {0, 0, 0};

length Arm = {200, 150};

Motor motor1 = {Motor1_Step_Pin, Motor1_Dir_Pin, 1};
Motor motor2 = {Motor2_Step_Pin, Motor2_Dir_Pin, 1};
Motor motor3 = {Motor3_Step_Pin, Motor3_Dir_Pin, 1};

Credentials credentials = {"SPG_IoT", "wgCtn3Hxh$w4g%NyMP3m*UEBiK^#AxBq7sRmEh!Rtz%46PR9%3"};

WebServer server(80);
