function [V1_] = nextposition(robotPose)

x = robotPose(1,1);
y = robotPose(1,2);
st = 4;

if robotPose(1,3) >= -0.3925 && robotPose(1,3) <= 0.3925
    V1_ = [x+st,y];
    % V3
%     V2 = [x+st,y-st];
%     V3 = [x,y-st];
%     V4 = [x,y+st];
%     V5 = [x+st,y+st];
end

if robotPose(1,3) > 0.3925 && robotPose(1,3) <= 1.1775
    V1_ = [x+st,y+st];
    %V2
%     V2 = [x+st,y];
%     V3 = [x+st,y-st];
%     V4 = [x-st,y+st];
%     V5 = [x, y+st];
end

if robotPose(1,3) > 1.1775 && robotPose(1,3) <= 1.9625
    V1_ = [x,y+st];
    %V1
%     V2 = [x+st,y+st];
%     V3 = [x+st,y];
%     V4 = [x-st,y];
%     V5 = [x-st,y+st];
end

if robotPose(1,3) > 1.9625 && robotPose(1,3) <= 2.7475
    V1_ = [x-st,y+st];
    %V8
%     V2 = [x,y+st];
%     V3 = [x+st,y+st];
%     V4 = [x-st,y-st];
%     V5 = [x-st,y];
end

if robotPose(1,3) > 2.7475 && robotPose(1,3) <= 3.1417
    V1_ = [x-st,y];
    %V7
%     V2 = [x-st,y+st];
%     V3 = [x,y+st];
%     V4 = [x,y-st];
%     V5 = [x-st,y-st];
end

if robotPose(1,3) > -3.1490 && robotPose(1,3) <= -2.7475
    V1_ = [x-st,y];
    %V7
%     V2 = [x-st,y+st];
%     V3 = [x,y+st];
%     V4 = [x,y-st];
%     V5 = [x-st,y-st];
end

if robotPose(1,3) > -1.1775 && robotPose(1,3) < -0.3925
    V1_ = [x+st,y-st];
    %V4
%     V2 = [x,y-st];
%     V3 = [x-st,y-st];
%     V4 = [x+st,y+st];
%     V5 = [x+st,y];
end

if robotPose(1,3) > -1.9625 && robotPose(1,3) <= -1.1775
    V1_ = [x,y-st];
    %V5
%     V2 = [x-st,y-st];
%     V3 = [x-st,y];
%     V4 = [x+st,y];
%     V5 = [x+st,y-st];
end

if robotPose(1,3) > -2.7475 && robotPose(1,3) <= -1.9625
    V1_ = [x-st,y-st];
    %V6
%     V2 = [x-st,y];
%     V3 = [x-st,y+st];
%     V4 = [x+st,y-st];
%     V5 = [x,y-st];
end



