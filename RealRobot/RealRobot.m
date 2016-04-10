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
        
        function scan=ultraScan(obj,num)
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
            
            
            dscan=[scanRaw(1:end-1,1) diff(scanRaw(:,2))];
            mask=abs(dscan(:,2))<5;
            mask=and(and(mask,circshift(mask,1)),circshift(mask,-1));
            scan=scanRaw(mask,:);
            scanSteps=round(size(scan,1)/obj.sensorRays);
            
            scan=scan(1:scanSteps:end,:);
            scan(:,1)=deg2rad(scan(:,1));
            
            %scan(:,2)
            %fitted=fitting2(scan(:,2));
            
            if nargin>1
                dlmwrite(['scanresult_' num2str(num) '.txt'],scanRaw)
                figure(100)
                plot(deg2rad(scanRaw(:,1)),scanRaw(:,2),'.')
                hold on
                plot(scan(:,1),scan(:,2),'.')
                plot(scan(:,1),fitted(:),'x')
                %plot(dscan(:,1),dscan(:,2),'.')
                hold off
                grid on
                axis([-pi/2 pi/2 -10 100]);
            end
            
            
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
    end
    
end

