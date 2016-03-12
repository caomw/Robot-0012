classdef RealRobot < handle
    %REALROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        h
        motor
        mSensor
        
        R
        w
        
    end
    
    methods
        
        function obj=RealRobot()
            obj.R=13/(2*pi);
            obj.w=12;
            
            COM_CloseNXT all 
            obj.h=COM_OpenNXT(); 
            COM_SetDefaultNXT(obj.h); 
            
            obj.motor=NXTMotor.empty(2,0);
            %left
            obj.motor(1)= NXTMotor('A', 'Power',0, 'TachoLimit',0,'ActionAtTachoLimit','Brake','SmoothStart',true);
            %right
            obj.motor(2)= NXTMotor('B', 'Power',0, 'TachoLimit',0,'ActionAtTachoLimit','Brake','SmoothStart',true);
            obj.motor(1).SendToNXT(); 
            obj.motor(2).SendToNXT();
            
            obj.mSensor= NXTMotor('C', 'Power', 50,'SpeedRegulation',false,'TachoLimit',0,'ActionAtTachoLimit','Brake','SmoothStart',true);
        end
        
        
        function move(obj,dist)
            theta=obj.moveTo(dist);
            obj.sendMotorCommand(theta,1);
        end
        function turn(obj,angle)
            theta=obj.turnTo(angle);
            obj.sendMotorCommand(theta,1);
        end
        
        function a=getSensAngle(obj)
            
            data=obj.mSensor.ReadFromNXT();
            a=-data.Position;
            
        end
        
        function scan=ultraScan(obj,wait)
            %TODO move around, and take mesaurements
            
            
            resolution = 5;
            %header1 = 'Measure in cm';
            %fid=fopen('SensorMovement.txt','w');
            %fprintf(fid, [ header1 '\n']);
            dist=-1*zeros(1000,1);
            angles=dist;
            direction=1;
            
            obj.moveSensor(-90,1);            
            obj.moveSensor(90,0); 
            angle=obj.getSensAngle();
            c=0;
            while angle<direction*90
                c=c+1;
                %obj.mSensor.SendToNXT(); 
                %obj.mSensor.WaitFor();
                %pause(1)
                angle=obj.getSensAngle();
                dist(c) = GetUltrasonic(SENSOR_1);
                angles(c)=angle;
                %fprintf(fid, '%f \n', R(c)');
            end
            %if wait
                obj.mSensor.WaitFor();
                pause(2);
            %end
            
            obj.mSensor.Power=50;
            %obj.mSensor.TachoLimit=resolution;
            
            %fclose(fid);
            NXT_PlayTone(500, 100);
            scan=[angles(dist~=-1) dist(dist~=-1)];
            
        end
        
        function sendMotorCommand(obj,theta,wait)
            for i=1:2
                obj.motor(i).Power=sign(theta(i))* 20;
                obj.motor(i).TachoLimit=round(abs(theta(1)));
                obj.motor(i).SendToNXT(); 
            end
            
            if wait
                for i=1:2
                obj.motor(i).WaitFor();
                end
            end
            
        end
        
        function moveSensor(obj,angle,wait)
            
            data=obj.mSensor.ReadFromNXT();
            angle=-angle;
            data.Position=data.Position;
            obj.mSensor.TachoLimit=abs(angle-data.Position);
            obj.mSensor.Power=50*sign(angle-data.Position);
            obj.mSensor.SendToNXT();
            
            if wait
                obj.mSensor.WaitFor();
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

