classdef RealRobot < handle
    %REALROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
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
            obj.motorPow=50;
            %left
            obj.motor(1)= NXTMotor('A', 'Power',0,'SpeedRegulation',true,'TachoLimit',0,'ActionAtTachoLimit','Brake','SmoothStart',true);
            %right
            obj.motor(2)= NXTMotor('B', 'Power',0,'SpeedRegulation',true,'TachoLimit',0,'ActionAtTachoLimit','Brake','SmoothStart',true);
            obj.motor(1).SendToNXT(); 
            obj.motor(2).SendToNXT();
            
            obj.mSensorPower=10;
            obj.mSensor= NXTMotor('C', 'Power', obj.mSensorPower,'SpeedRegulation',true,'TachoLimit',0,'ActionAtTachoLimit','Brake','SmoothStart',false);
            
            obj.sensorRays=20;
            obj.sensRange=75;
            obj.sensC=4.5;
        end
        
        
        function move(obj,dist)
            theta=obj.moveTo(dist);
            obj.sendMotorCommand(theta,1);
        end
        function turn(obj,angle)
            if abs(angle)>2*pi
                angle=deg2rad(angle);
            end
            theta=obj.turnTo(rad2deg(angle));
            obj.sendMotorCommand(theta,1);
        end
        
        function a=getSensAngle(obj)
            
            data=obj.mSensor.ReadFromNXT();
            a=-data.Position;
            
        end
        
        function scan=ultraScan(obj)
            %TODO move around, and take mesaurements
            
            
            resolution = 5;
            %header1 = 'Measure in cm';
            %fid=fopen('SensorMovement.txt','w');
            %fprintf(fid, [ header1 '\n']);
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
                %obj.mSensor.SendToNXT(); 
                %obj.mSensor.WaitFor();
                %pause(1)
                angle=obj.getSensAngle();
                dist(c) = GetUltrasonic(SENSOR_1);
                angle=(angle+obj.getSensAngle())/2;
                angles(c)=angle;
                %fprintf(fid, '%f \n', R(c)');
            end
            %if wait
            %angle
            obj.mSensor.WaitFor();
            obj.mSensor.Stop('off');
                %pause(2);
            %end
            
            
            %obj.mSensor.TachoLimit=resolution;
            
            %fclose(fid);
            obj.turnSensor(0,1,90); 
            %NXT_PlayTone(500, 100);
            
            scan=obj.condDataR(angles,dist);
            
            
        end
        
        function sendMotorCommand(obj,theta,wait)
            for i=1:2
                obj.motor(i).Power=sign(theta(i))* obj.motorPow;
                obj.motor(i).TachoLimit=round(abs(theta(1)));
                obj.motor(i).SendToNXT(); 
            end
            
            if wait
                for i=1:2
                    obj.motor(i).WaitFor();
                end
            end
            
        end
        
        function turnSensor(obj,angle,wait,pow)
            
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
            Z=obj.R*[.5 .5; 1/obj.w -1/obj.w];
            x(1,1)=0;
            x(2,1)=-deg2rad(angle);
            theta=Z\x;
            theta=rad2deg(theta);
        end
        
        function [ theta ] = moveTo(obj,dist )
            Z=obj.R*[.5 .5; 1/obj.w -1/obj.w];
            x(1,1)=dist;
            x(2,1)=0;
            theta=Z\x;
            theta=rad2deg(theta);
        end
        
        function scan=condDataR(obj,angles,dist,p)
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
            jPoints=find(abs(dscan(:,2))>distThreshold);
            if jPoints(1)~=1
                jPoints=[1;jPoints];
            end
            if jPoints(end)~=size(scanRaw,1)
                jPoints=[jPoints;size(scanRaw,1)];
            end
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



            %obj.sensorRays=20;
            scanSteps=round(size(scanRaw,1)/obj.sensorRays);
            %scanSteps=1;
            scan=scan(1:scanSteps:end,:);
            scan(:,1)=deg2rad(scan(:,1));

            %scan(:,2)
            %fitted=fitting2(scan(:,2));

            if nargin>3
                %dlmwrite(['scanresult_' num2str(num) '.txt'],scanRaw)
                figure
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
            
            
            scan(:,3)=sqrt(scan(:,3).^2+obj.sensC^2-2*scan(:,3)*obj.sensC*cos(pi-scan(:,1)));

        end
        
    end
    
end

