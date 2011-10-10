function [ ] = bug2( serPort )
%BUG2 implements the bug2 algorithm
%   Expects an initialized serPort to be passed
%   ex. serPort = RoombaInit(comPort);

td = 0.1; % time delta to wait
fs = 0.1; % forward speed
error = 0.5;

goal = [10, 0]; % assume goal is 10 meters in front of robot
pos = [0,0,0]; % [x,y,theta]

figure(1) % set the active figure handle to figure 1
clf; % clear figure 1
hold on; %Set figure 1 not to clear itself on each call to plot

while(1)
    % turn until we are pointing towards the goal
    
    % move towards goal
    d = DistanceSensorRoomba(serPort);
    [br,bl, wr,wl,wc, bf] = BumpsWheelDropsSensorsRoomba(serPort);
    hit = (bf==1 || br==1 || bl==1);
    SetFwdVelRadiusRoomba(serPort, fs, inf);
    while(hit == 0 && dist(pos, goal)>error)
        pause(td);
        [br,bl, wr,wl,wc, bf] = BumpsWheelDropsSensorsRoomba(serPort);
        hit = (bf==1 || br==1 || bl==1);
        d = DistanceSensorRoomba(serPort);
        pos(1) = pos(1) + d;
        plot(plot(1), plot(2), 'o');
    end
    % comments
    if(dist(pos, goal)< error)
        disp('Goal reached');
        break
    end
    SetFwdVelRadiusRoomba(serPort, 0, inf);
    d = DistanceSensorRoomba(serPort);
    pos(1) = pos(1) + d;
    if(dist(pos, goal)< error)
        disp('Goal reached');
        break
    end
    plot(plot(1), plot(2), 'o');

    % wall follow
    pos = wallfollower(pos, goal);
    if(pos == false)
        disp('Cannot reach goal');
        break
    elseif(dist(pos, goal)<error)
        disp('Goal reached');
        break
    end
    plot(plot(1), plot(2), 'o');
end

end

