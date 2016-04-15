function takeMeas(dist)
    samples=100;
    data=zeros(1,samples);
    for i=1:samples
        data(i)=GetUltrasonic(SENSOR_1);
    end
    mean(data)
    csvwrite([num2str(dist) 'cm.csv'],data);
end
