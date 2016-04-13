function condDataR(angles,dist)
    angles=angles(dist~=255);
    dist=dist(dist~=255);

    u=unique(angles);
    n=histc(angles,u);
    multiples=u(n>1);
    for i=1:numel(multiples)
        mask=angles==multiples(i);
        idx=find(mask);
        distM=mean(dist(mask));
        dist=dist(~mask);
        angles=angles(~mask);
        angles=[angles(1:idx(1)-1);multiples(i);angles(idx(1):end)];
        dist=[dist(1:idx(1)-1);distM;dist(idx(1):end)];
    end

    scanRaw=[angles(dist~=-1) dist(dist~=-1)];

    distThreshold=2;
    dscan=[scanRaw(1:end-1,1) diff(scanRaw(:,2))];
    jPoints=[1;find(abs(dscan(:,2))>distThreshold);size(scanRaw,1)];
    padding=1;
    numThreshold=5;
    angleThreshold=15;
    n=1;
    
    for i=1:length(jPoints)-1
        scanRange=scanRaw(jPoints(i)+padding:jPoints(i+1)-padding,:);
        %if size(scanRange,1)>numThreshold
        if abs(abs(scanRaw(jPoints(i)+padding,1))-abs(scanRaw(jPoints(i+1)-padding,1)))>angleThreshold
            points{n}=scanRange;%[scanRaw(jPoints(i:i+1),:)];
            n=n+1;
        end
    end
    
    
    scan=[];
    for i=1:length(points)
        x=points{i}([1 round(size(points{i},1)/2) end],1);
        y=points{i}([1 round(size(points{i},1)/2) end],2);
        p = polyfit(x,y,2);
        points{i}(:,3)=polyval(p,points{i}(:,1));
        scan=[scan;points{i}];
    end
    
    
    
    sensorRays=20;
    scanSteps=round(size(scanRaw,1)/sensorRays);
    scanSteps=1;
    scan=scan(1:scanSteps:end,:);
    scan(:,1)=deg2rad(scan(:,1));

    %scan(:,2)
    %fitted=fitting2(scan(:,2));

    if nargin>1
        %dlmwrite(['scanresult_' num2str(num) '.txt'],scanRaw)
        figure(plotE)
        plot(deg2rad(scanRaw(:,1)),scanRaw(:,2),'.')
        hold on
        plot(scan(:,1),scan(:,3),'.')
        %plot(scan(:,1),fitted(:),'x')
        %plot(dscan(:,1),dscan(:,2),'.')
        for i=1:length(jPoints)
            %scanRaw(jPoints(i),1)
            plot(deg2rad([scanRaw(jPoints(i),1) scanRaw(jPoints(i),1)]),[-10 100],'k--')
        end
        hold off
        grid on
        axis([-pi/2 pi/2 -10 100]);
    end


end