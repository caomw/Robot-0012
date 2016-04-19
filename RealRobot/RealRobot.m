classdef RealRobot < handle
    %REALROBOT Class for the real robot
    
    properties
        h
        motor
        motorPow
        mSensor
        mSensorPower
        sensorRays
        sensRange
        sensC
        
        R
        w
        
    end
    
    methods
        
        function obj=RealRobot()
            obj.R=13/(2*pi);
            obj.w=13;
            
            COM_CloseNXT all 
            obj.h=COM_OpenNXT(); 
            COM_SetDefaultNXT(obj.h); 
            OpenUltrasonic(SENSOR_1);
            
            obj.motor=NXTMotor.empty(2,0);
            obj.motorPow=100;
            %left
            obj.motor(1)= NXTMotor('A', 'Power',0,'SpeedRegulation',true,'TachoLimit',0,'ActionAtTachoLimit','Brake','SmoothStart',true);
            %right
            obj.motor(2)= NXTMotor('B', 'Power',0,'SpeedRegulation',true,'TachoLimit',0,'ActionAtTachoLimit','Brake','SmoothStart',true);
            obj.motor(1).SendToNXT(); 
            obj.motor(2).SendToNXT();
            
            obj.mSensorPower=80;
            obj.mSensor= NXTMotor('C', 'Power', obj.mSensorPower,'SpeedRegulation',true,'TachoLimit',0,'ActionAtTachoLimit','Brake','SmoothStart',false);
            
            obj.sensorRays=7;
            obj.sensRange=80;
            obj.sensC=4.5;
        end
        
        function victory(obj)
            NXT_PlayTone(1000, 120);
            pause(0.5);
            NXT_PlayTone(1000, 120);
        end
        
        
        function move(obj,dist)
            % move to distance
            theta=obj.moveTo(dist);
            obj.sendMotorCommand(theta,1);
        end
        
        function turn(obj,angle)
            % turn to angle
            if abs(angle)>2*pi
                angle=deg2rad(angle);
            end
            theta=obj.turnTo(rad2deg(angle));
            obj.sendMotorCommand(theta,1);
        end
        
        function a=getSensAngle(obj)    
            % get sensor angle
            data=obj.mSensor.ReadFromNXT();
            a=-data.Position;            
        end
        
        function scan=ultraScan(obj)
            % get ultrascan (angles, distances)
            % move back to 0 after the scan
            
            dist=-1*ones(200,1);
            angles=dist;
            direction=1;
            
            a_lim=obj.sensRange;
            obj.turnSensor(-a_lim,1,90);            
            obj.turnSensor(a_lim,0); 
            angle=obj.getSensAngle();
            c=0;
            while angle<direction*a_lim-5
                c=c+1;
                angle=obj.getSensAngle();
                dist(c) = GetUltrasonic(SENSOR_1);
                angle=(angle+obj.getSensAngle())/2;
                angles(c)=angle;
            end
            obj.mSensor.WaitFor();
            obj.mSensor.Stop('off');
            
            obj.turnSensor(0,1,90); 
            
            scan0=obj.condDataR(angles,dist);
            scan=scan0(:,[1 end]);
        end
        
<<<<<<< HEAD
        function scan=ultraScan2(obj)
            % get ultrascan (angles, distances)
            % don't move back
            
            dist=-1*ones(200,1);
            angles=dist;
            direction=-sign(obj.getSensAngle());
            if direction==0
                direction=1;
                obj.turnSensor(-obj.sensRange,1,90);
            end
            
            a_lim=obj.sensRange*direction;       
            obj.turnSensor(a_lim,0); 
            angle=obj.getSensAngle();
            c=0;
            while abs(a_lim-angle)>1
                c=c+1;
                angle=obj.getSensAngle();
                dist(c) = GetUltrasonic(SENSOR_1);
                angle=(angle+obj.getSensAngle())/2;
                angles(c)=angle;
            end
            obj.mSensor.WaitFor();
            obj.mSensor.Stop('off');
            
            scan0=obj.condDataR(angles,dist);
            
            scan=scan0(:,[1 end]);
        end
        
=======
>>>>>>> eaede6fc837dd694d2b8526f65c5fbd89cf2033b
        function sendMotorCommand(obj,theta,wait)
            % send commands to motors
            
            for i=1:2
                if round(abs(theta(i)))~=0
                    obj.motor(i).Stop('off');
                    obj.motor(i).Power=sign(theta(i))* obj.motorPow;
                    obj.motor(i).TachoLimit=round(abs(theta(i)));
                    obj.motor(i).SendToNXT(); 
                end
            end
            
            if wait
                for i=1:2
                    obj.motor(i).WaitFor();
                    obj.motor(i).Stop('off');
                end
            end
            
        end
        
        function turnSensor(obj,angle,wait,pow)
            % turn the sensor to a position
            
            data=obj.mSensor.ReadFromNXT();
            angle=-angle;
            data.Position=data.Position;
            obj.mSensor.TachoLimit=abs(angle-data.Position);
            if nargin>3
                obj.mSensor.Power=pow*sign(angle-data.Position);
            else
                obj.mSensor.Power=obj.mSensorPower*sign(angle-data.Position);
            end
            obj.mSensor.SendToNXT();
            
            if nargin>2
                if wait
                    obj.mSensor.WaitFor();
                    obj.mSensor.Stop('off');
                end
            end
        end
        
        function [ theta ] = turnTo(obj, angle )
            % calculate motor commands for turning
            
            Z=obj.R*[.5 .5; 1/obj.w -1/obj.w];
            x(1,1)=0;
            x(2,1)=-deg2rad(angle);
            theta=Z\x;
            theta=rad2deg(theta);
        end
        
        function [ theta ] = moveTo(obj,dist )
            % calculate motor commands for moving
            
            Z=obj.R*[.5 .5; 1/obj.w -1/obj.w];
            x(1,1)=dist;
            x(2,1)=0;
            theta=Z\x;
            theta=rad2deg(theta);
        end
        
        function scan=condDataR0(obj,angles,dist,p)
            % data conditioning 0 (experimental)
            
            
            angles=angles(dist<90);
            p =[0.000000060758403  -0.000012216779499  -0.000142367747446   1.102508214926583  -1.035450741947817];
            
            dist=dist(dist<90);
            scan(:,2)=dist(dist~=-1);
            %dist=polyval(p,dist);
            %{
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
            %}

            scanRaw=[deg2rad(angles(dist~=-1)) dist(dist~=-1)];

            
            offset=0;
            gamma=(scanRaw(:,1));
            a=scanRaw(:,2)+offset;
            b=obj.sensC;
            c=sqrt(a.^2+b^2-2*a.*b.*cos(pi-abs(gamma)));
            %sqrt((scan(:,3)+2).^2+obj.sensC^2-2*scan(:,3)*obj.sensC.*cos(pi-abs(scan(:,1))));
            alpha=acos((a.^2-b.^2-c.^2)./(-2*c*b));
            
            
            scan(:,3)=c;
            scan(:,1)=alpha.*sign(gamma);
            

        end
        
        function scanOut=condDataR(obj,angles,dist,p)
            % data conditioning (final)
            
            angles=angles(dist<80);
            dist=dist(dist<80);
            %{
            angles=angles(dist<80);
            p =[0.000000060758403  -0.000012216779499  -0.000142367747446   1.102508214926583  -1.035450741947817];
            angles=polyval(p,angles);
            dist=dist(dist<80);
            %}
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

            scanRaw=[deg2rad(angles(dist~=-1)) dist(dist~=-1)];

            distThreshold=2;
            dscan=[scanRaw(1:end-1,1) diff(scanRaw(:,2))];
            jPoints=find(abs(dscan(:,2))>distThreshold);
            if ~isempty(jPoints)
                if jPoints(1)~=1
                    jPoints=[1;jPoints];
                end
                if jPoints(end)~=size(scanRaw,1)
                    jPoints=[jPoints;size(scanRaw,1)];
                end
            else
                jPoints=[1;size(scanRaw,1)];
            end
                
            padding=1;
            numThreshold=5;
            angleThreshold=deg2rad(10);
            n=1;
            points={};
            for i=1:length(jPoints)-1
                scanRange=scanRaw(jPoints(i)+padding:jPoints(i+1)-padding,:);
                %if size(scanRange,1)>numThreshold
                if size(scanRange,1)>2
                    %if abs(abs(scanRaw(jPoints(i)+padding,1))-abs(scanRaw(jPoints(i+1)-padding,1)))>angleThreshold
                    if abs(scanRaw(jPoints(i+1)-padding,1)-scanRaw(jPoints(i)+padding,1))>angleThreshold
                        points{n}=scanRange;%[scanRaw(jPoints(i:i+1),:)];
                        n=n+1;
                    end
                end
            end
            if isempty(points)
                points{1}=1;
            end
            
            scan=[];
            for i=1:length(points)
                %points{i}
                x=points{i}([1 round(size(points{i},1)/2) end],1);
                y=points{i}([1 round(size(points{i},1)/2) end],2);
                p = polyfit(x,y,2);
                points{i}(:,3)=polyval(p,points{i}(:,1));
                scan=[scan;points{i}];
            end

            scanSteps=round(size(scanRaw,1)/obj.sensorRays);
            scan=scan(1:scanSteps:end,:);

            
            offset=0;
            gamma=(scan(:,1));
            a=scan(:,3)+offset;
            b=obj.sensC;
            c=sqrt(a.^2+b^2-2*a.*b.*cos(pi-abs(gamma)));
            alpha=acos((a.^2-b.^2-c.^2)./(-2*c*b));
            
            
            scan(:,3)=c;
            scan(:,1)=alpha.*sign(gamma);
            sides=6;
            scanOut=[scanRaw(1:sides/2:sides,:);scan(:,[1 3]);scanRaw(end-sides:sides/2:end,:)];

        end
        
        function scan=condDataR2(obj,angles,dist,p)
            % data conditioning 2 (experimental)
            
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
            scanRaw(:,1)=deg2rad(scanRaw(:,1));

            distThreshold=1;
            dscan=[scanRaw(1:end-1,1) diff(scanRaw(:,2))];
            jPoints=find(abs(dscan(:,2))>distThreshold);
            if ~isempty(jPoints)
                if jPoints(1)~=1
                    jPoints=[1;jPoints];
                end
                if jPoints(end)~=size(scanRaw,1)
                    jPoints=[jPoints;size(scanRaw,1)];
                end
            else
                jPoints=[1;size(scanRaw,1)];
            end
                
            padding=1;
            numThreshold=5;
            angleThreshold=deg2rad(15);
            n=1;
            points={};
            for i=1:length(jPoints)-1
                scanRange=scanRaw(jPoints(i)+padding:jPoints(i+1)-padding,:);
                %if size(scanRange,1)>numThreshold
                if size(scanRange,1)>2
                    %if abs(abs(scanRaw(jPoints(i)+padding,1))-abs(scanRaw(jPoints(i+1)-padding,1)))>angleThreshold
                    if abs(scanRaw(jPoints(i+1)-padding,1)-scanRaw(jPoints(i)+padding,1))>angleThreshold
                        points{n}=scanRange;%[scanRaw(jPoints(i:i+1),:)];
                        n=n+1;
                    end
                end
            end
            if isempty(points)
                points{1}=1;
            end
            
            scan=[];
            lines=[];
            n=1;
            thresholdN=5;
            i=1;
            pointsTemp=points;
            while i<=length(points)
                m=min(pointsTemp{i}(:,2));
                idx=find(pointsTemp{i}(:,2)==m);

                if length(idx)<=thresholdN
                    pointsTemp{i}(idx,2)=NaN;
                    %i=i-1;
                    if isempty(idx)
                        i=i+1;
                    end
                else
                    if idx(1)~=1 && idx(end)~=length(pointsTemp{i}(:,2))
                        d=m;
                        alpha=meanangle(pointsTemp{i}(idx,1));
                        betas=(pointsTemp{i}(:,1));
                        linePoints=d./cos(abs(alpha-betas));
                        points{i}(:,3)=linePoints;
                        scan=[scan;points{i}];
                    else
                        points{i}(:,3)=points{i}(:,2);
                        scan=[scan;points{i}];
                    end
                    i=i+1;
                end
            end

            scanSteps=round(size(scanRaw,1)/obj.sensorRays);
            scanSteps=1;
            scan=scan(1:scanSteps:end,:);
            
            scan(:,3)=sqrt((scan(:,3)+2).^2+obj.sensC^2-2*scan(:,3)*obj.sensC.*cos(pi-scan(:,1)));
        end
        
        
    end
    
end

