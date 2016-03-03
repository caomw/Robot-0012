      % MOTOR A AND B POWER 60 AND ONLY 1000 
      m = NXTMotor('AB', 'Power', 60);
      m.TachoLimit         = 1000;
      m.SmoothStart        = true,    % start soft
      m.ActionAtTachoLimit = 'coast'; % we want very smooth "braking", too :
      s = 10;
      %Repeat the movement S times and record the sensor measure to R
      for c = 1:s
        m.SendToNXT();
        R(c) = GetUltrasonic(SENSOR_1)
      end

      m.WaitFor();                    % are we there yet?

      % we're here, motors are still moving / coasting, so give the bot time!
      pause(3);

      % you can still hear the synchronization (high noisy beeping)
      % before we go back, we have to disable the synchronization quickly
      m.Stop();

      % reverse direction
      m.Power = -m.Power;
      m.SendToNXT();
      m.WaitFor();
      pause(3);
      m.Stop();

      NXT_PlayTone(500, 100);