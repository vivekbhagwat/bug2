function [ ] = bug2( serPort )
%BUG2 implements the bug2 algorithm
%   Expects an initialized serPort to be passed
%   ex. serPort = RoombaInit(comPort);

goalError = 0.2; % distance from goal to hit
angleError = 0.1; % radians, before we start going towards goal

if isSimulator(serPort)
    td = 0.1; % time delta to wait
    fs = 0.1; % forward speed
    as = 0.2; % angle speed (0, 0.2 m/s)    
    corrective = 1.0; %.5; % how much to fix the angle deltas by
else
    td = 0.01;
    fs = 0.1;
    as = 0.05;
    corrective = 1.5; 
end

goal = [10, 0]; % assume goal is 10 meters in front of robot
pos = [0,0,0]; % [x,y,theta]

% drawing initialization
figure(1) % set the active figure handle to figure 1
clf; % clear figure 1
axis([0 10 -5 5]);
hold on; %Set figure 1 not to clear itself on each call to plot

while(1)
    % turn until we are pointing towards the goal
    AngleSensorRoomba(serPort);
    while(abs(pos(3)) > angleError)
        turnAngle(serPort, as, -pos(3)/4);
        angle = AngleSensorRoomba(serPort);
        disp(angle);
        pos(3) = pos(3) + corrective*angle;
    end
    
    DistanceSensorRoomba(serPort); % clear distance
    [br,bl, ~,~,~, bf] = BumpsWheelDropsSensorsRoomba(serPort);
    hit = (bf==1 || br==1 || bl==1);
    % move towards goal
    SetFwdVelRadiusRoomba(serPort, fs, inf);
    while(hit == 0 && dist(pos, goal)>goalError)
        % go for a little while
        pause(td);
        % poll the bumpers
        [br,bl, wr,wl,wc, bf] = BumpsWheelDropsSensorsRoomba(serPort);
        % if picked up, kill
        if (wr == 1 || wl == 1 || wc == 1)
            SetFwdVelRadiusRoomba(serPort, 0, 0);
            return;
        end
        hit = (bf==1 || br==1 || bl==1);
        d = DistanceSensorRoomba(serPort);
        pos(1) = pos(1) + d;
        % draw current location
        plot(pos(1), pos(2), 'o');
    end
    SetFwdVelRadiusRoomba(serPort, 0, inf);
    % check if close to goal
    if(dist(pos, goal)< goalError)
        disp('Goal reached');
        break
    end
    d = DistanceSensorRoomba(serPort);
    pos(1) = pos(1) + d;
    % check again: stops aren't atomic
    if(dist(pos, goal)< goalError)
        disp('Goal reached');
        break
    end
    plot(pos(1), pos(2), 'o');

    % wall follow
    pos = wall_follower(serPort, pos, goal);
    % assume we aren't blocked
    % check if we can reach the goal, or whether we reached the goal
    if(pos == false)
        disp('Cannot reach goal');
        break
    elseif(dist(pos, goal)<goalError)
        disp('Goal reached');
        break
    end
    % draw location
    plot(pos(1), pos(2), 'o');
end

end

