for i=1:10
    dist(i)=GetUltrasonic(SENSOR_1);
    robot.move(-5);
end
