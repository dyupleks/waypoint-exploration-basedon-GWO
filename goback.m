function [Vback] = goback(robotPose)

x = robotPose(1,1);
y = robotPose(1,2);
st = 2;

if robotPose(1,3) >= -0.3925 && robotPose(1,3) <= 0.3925
    Vback = [x-st,y];
    
    % if robot in V3, go V7

end

if robotPose(1,3) > 0.3925 && robotPose(1,3) <= 1.1775
    Vback = [x-st,y-st];
    %if robot in V2, go V6

end

if robotPose(1,3) > 1.1775 && robotPose(1,3) <= 1.9625
    Vback = [x,y-st];
    % id a robot in V1, go V5

end

if robotPose(1,3) > 1.9625 && robotPose(1,3) <= 2.7475
    Vback = [x+st,y-st];
    %if robot in V8, go V4

end

if robotPose(1,3) > 2.7475 && robotPose(1,3) <= 3.1417
    Vback = [x+st,y];
    %if robot in V7, go V3
end

if robotPose(1,3) > -3.1490 && robotPose(1,3) <= -2.7475
    Vback = [x+st,y];
    %if robot in V7, go V3

end

if robotPose(1,3) > -1.1775 && robotPose(1,3) < -0.3925
    Vback = [x-st,y+st];
    %if robot in V4, go V8

end

if robotPose(1,3) > -1.9625 && robotPose(1,3) <= -1.1775
    Vback = [x,y+st];
    %if robot in V5, go V1

end

if robotPose(1,3) > -2.7475 && robotPose(1,3) <= -1.9625
    Vback = [x+st,y+st];
    %if robot in V6, go V2

end