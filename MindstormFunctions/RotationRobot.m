%Result is movement 10 cm NXTMotor('A', 'Power', -50, 'TachoLimit', 360)
%Result is movement 5 cm NXTMotor('A', 'Power', -50, 'TachoLimit', 180)
%MOTOR A IS LEFT AND MOTOR B IS RIGHT
theta = moveTo([1;360])
mLeft= NXTMotor('A', 'Power',sign(theta(1))* 50, 'TachoLimit',round(abs(theta(1))));
mRight= NXTMotor('B', 'Power', sign(theta(2))*50, 'TachoLimit', round(abs(theta(2))));
mLeft.SendToNXT(); 
mRight.SendToNXT();
GetUltrasonic(SENSOR_1)



