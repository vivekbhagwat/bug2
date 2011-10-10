function [ ] = bug2( serPort )
%BUG2 implements the bug2 algorithm
%   Expects an initialized serPort to be passed
%   ex. serPort = RoombaInit(comPort);

td = 0.1; % time delta to wait
fs = 0.1; % forward speed
as = 0.05; % angle speed (0, 0.2 m/s)
goalError = 0.5; % distance from goal to hit
angleError = 0.1; % radians, before we start going towards goal

goal = [10, 0]; % assume goal is 10 meters in front of robot
pos = [0,0,0]; % [x,y,theta]

% drawing initialization
figure(1) % set the active figure handle to figure 1
clf; % clear figure 1
hold on; %Set figure 1 not to clear itself on each call to plot

while(1)
    % turn until we are pointing towards the goal
    AngleSensorRoomba(serPort);
    while(abs(pos(3)) > angleError)
        turnAngle(serPort, as, -pos(3)/2);
        pos(3) = pos(3) + AngleSensorRoomba(serPort);
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
        [br,bl, ~,~,~, bf] = BumpsWheelDropsSensorsRoomba(serPort);
        hit = (bf==1 || br==1 || bl==1);
        d = DistanceSensorRoomba(serPort);
        pos(1) = pos(1) + d;
        % draw current location
        plot(plot(1), plot(2), 'o');
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
    plot(plot(1), plot(2), 'o');

    % wall follow
    pos = wall_follower(pos, goal);
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
    plot(plot(1), plot(2), 'o');
end

end

