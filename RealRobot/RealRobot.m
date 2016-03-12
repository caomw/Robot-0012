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
            
            %left
            obj.motor(1)= NXTMotor('A', 'Power',0, 'TachoLimit',0,'ActionAtTachoLimit','Brake');
            %right
            obj.motor(2)= NXTMotor('B', 'Power',0, 'TachoLimit',0,'ActionAtTachoLimit','Brake');
            obj.motor(1).SendToNXT(); 
            obj.motor(2).SendToNXT();
            
            obj.mSensor= NXTMotor('C', 'Power', 50,'SpeedRegulation',false,'TachoLimit',0,'ActionAtTachoLimit','Brake',SmoothStart,true);
        end
        
        
        function move(obj,dist)
            theta=moveTo(dist);
            senMotorCommand(theta,1);
        end
        function turn(obj,angle)
            theta=turnTo(angle);
            senMotorCommand(theta,1);
        end
        
        function scan=ultraScan(obj,wait)
            %TODO move around, and take mesaurements
            
            
            Resolution = 5;
            %header1 = 'Measure in cm';
            %fid=fopen('SensorMovement.txt','w');
            %fprintf(fid, [ header1 '\n']);
            dist=-1*zeros(1000,1);
            angles=dist;
            direction=1;
            
            data=obj.mSensor.ReadFromNXT();

            obj.mSensor.TachoLimit=abs(-90-data.Position);
            obj.mSensor.Power=50*sign(-90-data.Position);
            obj.mSensor.SendToNXT();
            %if wait
                obj.mSensor.WaitFor();
                pause(2);
            %end
            obj.mSensor.Power=50;
            obj.mSensor.TachoLimit=Resolution;

            c=0;
            while data.Position<direction*90
                c=c+1;
                obj.mSensor.SendToNXT(); 
                obj.mSensor.WaitFor();
                pause(1)
                data=obj.mSensor.ReadFromNXT();
                dist(c) = GetUltrasonic(SENSOR_1);
                angles(c)=data.Position;
                %fprintf(fid, '%f \n', R(c)');
            end
            %fclose(fid);
            NXT_PlayTone(500, 100);
            scan=[angles(dist~=-1) dist(dist~=-1)];
            
        end
        
        function sendMotorCommand(obj,theta,wait)
            for i=1:2
                obj.motor(i).Power=sign(theta(i))* 50;
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

            obj.mSensor.TachoLimit=abs(angle-data.Position);
            obj.mSensor.Power=50*sign(angle-data.Position);
            obj.mSensor.SendToNXT();
            
            if wait
                obj.mSensor.WaitFor();
            end
        end
        
        function [ theta ] = turnTo(obj, angle )
            Z=obj.R*[.5 .5; 1/obj.w -1/obj.w];
            x(1)=0;
            x(2)=deg2rad(angle);
            theta=Z\x;
            theta=rad2deg(theta);
        end
        
        function [ theta ] = moveTo(obj,dist )
            Z=obj.R*[.5 .5; 1/obj.w -1/obj.w];
            x(1)=dist;
            x(2)=0;
            theta=Z\x;
            theta=rad2deg(theta);
        end
    end
    
end

