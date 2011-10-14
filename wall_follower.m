function return_list = wall_follower(serPort, q_hit, q_goal)
%should return current_position array [x,y,theta]
%returns false if reaches where it was before.

% angle in degrees to turn
th = 5;

% speed
if isSimulator(serPort)
    % time delays
    tdd = 0.5;
    gs = 0.2; % general speed
    ts = 0.2; % turning speed
    corrective = 1.0 ;%.5; % how much to fix the angle deltas by
    corrective2 = 1.0; % how much to fix, when bumping into wall
else
    tdd = 0.01;
    gs = 0.1;
    ts = 0.05;
    corrective = 2.0; 
    corrective2 = 2.0; 
end

thresh = 0.05; % how far away you need to move before returning

% Assume we're already touching the object

% have to keep track of where it started
% this is nowhere near perfect, but it should give a reasonable approx
DistanceSensorRoomba(serPort);
AngleSensorRoomba(serPort);

origin_x = q_hit(1);
origin_y = q_hit(2);
origin_angle = q_hit(3);

goal_x = q_goal(1);
goal_y = q_goal(2);

x = origin_x;
y = origin_y;
angle = origin_angle;

ret = 0; % if we've moved far enough away

BOOL = true; % check if we've touched the line
while(not(dist_point_to_line([x,y],[origin_x,origin_y],[goal_x,goal_y]) < thresh && ret == 1)) 
    display(sprintf('<x:%f y:%f> - <hit_x: %f hit_y:%f>', x,y, origin_x, origin_y));
    plot(x, y, 'o');
    [br,bl, wr,wl,wc, bf] = BumpsWheelDropsSensorsRoomba(serPort);
    
    % turn until not bumping wall
    % always turn counter-clockwise
    
    AngleSensorRoomba(serPort);
    while(bf==1 || br==1 || bl ==1 && not(dist_point_to_line([x,y],[origin_x,origin_y],[goal_x,goal_y]) < thresh && ret==1))
        if (wr || wl || wc)
            break;
        end
        % check if we've returned
        if(dist_point_to_line([x,y],[origin_x,origin_y],[goal_x,goal_y]) < thresh && ret==1)
            continue;
        elseif(dist([x,y],[origin_x,origin_y]) > 2*thresh && ret==0)
            ret = 1;
        end

        % retain finer resolution when not head-on
        if(bf==1)
            turnAngle(serPort, ts, th);
        else
            turnAngle(serPort, ts, th/2);
        end
        a = AngleSensorRoomba(serPort);
        angle = angle + corrective2*a;
        [br,bl, wr,wl,wc, bf] = BumpsWheelDropsSensorsRoomba(serPort);
    end
    a = AngleSensorRoomba(serPort);
    angle = angle + corrective2*a;

    
    % move, turn (clockwise) until touch the wall again
    i = 0;
    while(bf==0 && br==0 && bl==0 && BOOL)
        if (wr == 1 || wl == 1 || wc == 1)
            break;
        end
        
        display(sprintf('<(2) %f>', dist([x,y],[origin_x,origin_y])));
        display(sprintf('<(3) %f>', dist_point_to_line([x,y],[origin_x,origin_y],[goal_x,goal_y])));
        plot(x, y, 'o');
        
        % check if we've returned
        if(dist_point_to_line([x,y],[origin_x,origin_y],[goal_x,goal_y]) < thresh && ret==1)
            BOOL = false;
            continue;
        elseif(dist([x,y],[origin_x,origin_y]) > 2*thresh && ret==0)
            ret = 1;
        end
        
        % not using travelDist because it gets stuck in corners
        SetFwdVelRadiusRoomba(serPort, gs, inf);
        pause(tdd);
        SetFwdVelRadiusRoomba(serPort, 0, inf);
        % update distance traveled
        b = DistanceSensorRoomba(serPort);
        a = AngleSensorRoomba(serPort);
        angle = angle + corrective*a;
        x = x + b*cos(angle);
        y = y + b*sin(angle);
        % check if we've hit
        [br,bl, wr,wl,wc, bf] = BumpsWheelDropsSensorsRoomba(serPort);
        if(bf==0 && br==0 && bl==0)
            turnAngle(serPort,  ts, -th*(1+i*0.1));
%             i = i+1;
            [br,bl, wr,wl,wc, bf] = BumpsWheelDropsSensorsRoomba(serPort);
        end
    end
    
    a = AngleSensorRoomba(serPort);
    angle = angle + corrective*a;
    
    if (wr == 1 || wl == 1 || wc == 1)
        break;
    end
    
	% pause(0.5);
	display(sprintf('%f %f %f', bf, br, bl));
end

% stop the robot if we broke the loop
SetFwdVelRadiusRoomba(serPort, 0, inf);

display('Finished: back at starting point');
pause(1);

return_list = [x,y,angle];
