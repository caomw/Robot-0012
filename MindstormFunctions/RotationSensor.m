
Resolution = 5;
%header1 = 'Measure in cm';
%fid=fopen('SensorMovement.txt','w');
%fprintf(fid, [ header1 '\n']);
R2=zeros(1000,1);
angles2=R;
direction=1;
mSensor= NXTMotor('C', 'Power', direction*50);
mSensor.SpeedRegulation = false;
mSensor.TachoLimit = Resolution;
mSensor.ActionAtTachoLimit = 'Brake';
mSensor.SmoothStart = true;
data=mSensor.ReadFromNXT();

mSensor.TachoLimit=abs(-90-data.Position);
mSensor.Power=50*sign(-90-data.Position);
mSensor.SendToNXT();
%mSensor.WaitFor();
pause(2);
mSensor.Power=50;
mSensor.TachoLimit=Resolution;

c=0
while data.Position<direction*90
    c=c+1;
    mSensor.SendToNXT(); 
    mSensor.WaitFor();
    pause(1)
    data=mSensor.ReadFromNXT();
    R2(c) = GetUltrasonic(SENSOR_1);
    angles2(c)=data.Position;
    %fprintf(fid, '%f \n', R(c)');
end
%fclose(fid);
NXT_PlayTone(500, 100);

