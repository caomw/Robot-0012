function moveSensor(angle)

    mSensor= NXTMotor('C', 'Power', 50);
    mSensor.SpeedRegulation = false;
    mSensor.TachoLimit = 0;
    mSensor.ActionAtTachoLimit = 'Brake';
    mSensor.SmoothStart = true;
    data=mSensor.ReadFromNXT();

    mSensor.TachoLimit=abs(angle-data.Position);
    mSensor.Power=50*sign(angle-data.Position);
    mSensor.SendToNXT();
end