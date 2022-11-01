clc;
clear;
close all;
% rosinit

mapsize = 20;
inflatesize = 0.5;
nItern = 2300;
dim = 2;
varSize = [1 dim];

map_ = robotics.BinaryOccupancyGrid(mapsize+10,mapsize,2);
%%
% obstacle's coordinate

A(1:60) = 0:inflatesize:29.8;
B(1:60) = 0.4;
C = [A;B];
E = [B(1:40);A(1:40)];
C = transpose(C);
C(11:20,:) = 0;
E = transpose(E);
F(1:60) = 19.7;
G = [A;F];
G = transpose(G);
Ff(1:40) = 29.7;
H = [Ff;A(1:40)];
H = transpose(H);
I(1:20) = 20:inflatesize:29.5;
J(1:20) = 12;
K = [I;J];
K = transpose(K);
L(1:10) = 5;
Ii(1:10) = 20:inflatesize:24.5;
M = [Ii;L];
M = transpose(M);

N(1:12) = 9;
O(1:12) = 0:inflatesize:5.5;
P = [N;O];
P = transpose(P);
S(1:10) = 15:inflatesize:19.5;
Nn = N(1:10);
T = [Nn;S];
T = transpose(T);


% C is below horizontal wall
setOccupancy(map_, C, 1)
% E is left vertical wall
setOccupancy(map_, E, 1)
% G is upper horizontal wall
setOccupancy(map_, G, 1)
% H is right vertical wall
setOccupancy(map_, H, 1)
% obstacles inside environment
setOccupancy(map_, P, 1)
setOccupancy(map_, T, 1)
inflate(map_, 0.3)
% thin obstacles
setOccupancy(map_, K, 1)
setOccupancy(map_, M, 1)
inflate(map_, 0.3)



%%
% input parameter of ExampleHelperRobotSimulator should be
% BinaryOccupancyGrid object
robot = ExampleHelperRobotSimulator(map_);
grid on;
% 
setRobotPose(robot,[3 4 0]);
enableROSInterface(robot,true);
robot.LaserSensor.NumReadings = 50;
% enableLaser(sim,true);
% showTrajectory (sim,true);
%%

% we need to get simulator laser data. that's why we should subscribe the
% scan topic
scanSub = rossubscriber('scan');

% publisher for sending velocity command
[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');
path = [5 5];
controller = robotics.PurePursuit('Waypoints', path);
controller.DesiredLinearVelocity = 0.8;
controller.MaxAngularVelocity = 0.8;
% rostf allows to find robot pose at the time when laser is observing
tftree = rostf;
pause(1)

map = robotics.OccupancyGrid(mapsize+10,mapsize,10);
figureHandle = figure('Name', 'Map');
axesHandle = axes('Parent', figureHandle);
mapHandle = show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Update 0');

controlRate = robotics.Rate(10);
% value of laser sensor
group1 = robot.LaserSensor.AngleSweep(1:1:5);
% laser index
group1_inx = 1:5;

% number is negative, sharp right side
group2 = robot.LaserSensor.AngleSweep(6:1:20);
group2_inx = 6:20;

% number is negative, right side
group3 = robot.LaserSensor.AngleSweep(21:1:30);
group3_inx = 21:30;

% number is positive, left side
group4 = robot.LaserSensor.AngleSweep(31:1:45);
group4_inx = 31:45;

% number is positive, sharp left side
group5 = robot.LaserSensor.AngleSweep(46:1:50);
group5_inx = 46:50;

V1_ = [];

globalPop.Pose = [];
globalPop.Cost = [];
del_list = [];
del_cost = [];


point = [];

a = 2;
adamp = a/nItern;
countPts = 1; 
cnt = 1;
cnt_=1;
alpha.Pose = [];
alpha.Cost = [];
alpha.Ind = [];
beta.Pose = [];
beta.Cost = [];
beta.Ind = [];
gamma.Pose = [];
gamma.Cost = [];
gamma.Ind = [];
BestPosition = [];
steps = 0;
gwopointcounter = 0;
Vback = [];
angleObs = [0, pi/4, pi/2, 3*pi/4, pi, 5*pi/4, 3*pi/2, 7*pi/4];
maxrangeObs = 1;
poseArray = [3 4 0];
poseArrDist = 0;
evalObstCounter = [];
exploredness = [];


for j=1:nItern
%     if nItern == 252 || nItern == 1002
%         figcap = screencapture();
%     end
    
%     if exploredness(j) = 1, it explores new cells
    exploredness(j) = 0; 
    obs_ranges = [];
    occArr = [];
    group1V3_flag = false;
    group2V2_flag = false;
    group3V1_flag = false;
    group4V5_flag = false;
    group5V4_flag = false;
    
    scanMsg = receive(scanSub);
    
    % transform from robot_base to map
    pose = getTransform(tftree, 'map', 'robot_base', scanMsg.Header.Stamp, 'Timeout', 2);
    position = [pose.Transform.Translation.X, pose.Transform.Translation.Y];
    orientation =  quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, ...
                   pose.Transform.Rotation.Y, pose.Transform.Rotation.Z], 'ZYX');
    robotPose = [position, orientation(1)];
    x = robotPose(1,1);
    y = robotPose(1,2);
    
    scan = lidarScan(scanMsg);
    ranges = scan.Ranges;
    ranges(isnan(ranges)) = robot.LaserSensor.MaxRange;
    modScan = lidarScan(ranges, scan.Angles);
    rangesNotNan = scan.Ranges;
    rangesNotNan(isnan(rangesNotNan)) = 5;
    
    
        % Visualize the map after every 50th update.
    if ~mod(j,1)
        mapHandle.CData = occupancyMatrix(map);
        title(axesHandle, ['OccupancyGrid: Update ' num2str(j)]);
    end
    
    % find neighbor cells around a robot according to orientation value
    [V1_] = nextposition(robotPose);
%     hold on;
%     plot(V1(1,1), V1(1,2), 'Marker', 'o');
    
    for i=1:50
        if ranges(i)~=5         
            obs_ranges = [obs_ranges, i];             
        end
    end
   
   
    % if a laser touches obstacle, return flag true
   for i=1:1:length(obs_ranges) 
        if ismember(obs_ranges(i),group1_inx(1:length(group1_inx)))
            group1V3_flag = true;
        end
        if ismember(obs_ranges(i),group2_inx(1:length(group2_inx)))
            group2V2_flag = true;
        end
        % straight
        if ismember(obs_ranges(i),group3_inx(1:length(group3_inx)))
            group3V1_flag = true;
        end
        if ismember(obs_ranges(i),group4_inx(1:length(group4_inx)))
            group4V5_flag = true;
        end
        if ismember(obs_ranges(i),group5_inx(1:length(group5_inx)))
            group5V4_flag = true;
        end
   end
   
   countPts = length(globalPop);
     % raycast returns nan when the end is obstacle
     % globalPop doesnt have obstacle points
    for i=1:50
        
            sensor_count = robot.getRangeData;
            
            if isnan(sensor_count(i))
                
                    [endPts] = raycast(map,robotPose, rangesNotNan(i), robot.LaserSensor.AngleSweep(i));
                    if ~isempty(endPts)
                        if j == 1
                            temp_value = grid2world(map, endPts);
                            globalPop(countPts).Pose = [temp_value(1,1), temp_value(1,2)];   
                            % the costs are only for new frontier points  
                            globalPop(countPts).Cost = pdist([[robotPose(1),robotPose(2)];[temp_value(1,1), temp_value(1,2)]],'euclidean');
                            countPts = countPts + 1;
                        else
                            temp_value = grid2world(map, endPts);
                            globalPop(countPts+1).Pose = [temp_value(1,1), temp_value(1,2)];   
                            % the costs are only for new frontier points  
                            globalPop(countPts+1).Cost = pdist([[robotPose(1),robotPose(2)];[temp_value(1,1), temp_value(1,2)]],'euclidean');
                            countPts = countPts + 1;
                        end
                        expArray = getOccupancy(map,[temp_value(1,1), temp_value(1,2)]);
                        if expArray >= 0.5
                           exploredness(j) = 1;  
                        end
                    end
            end   
        
    end
    
           
    % remove not frontier points starting after first iteration
    % if one point was frontier before , but now it is explored
    if j>1
        
        % filters
        % 1. delete points from globalPop array if they are explored
        for i = 1:length(globalPop)
            occ = getOccupancy(map,[globalPop(i).Pose(1,1), globalPop(i).Pose(1,2)]);           
            if occ<0.5
                del_list(cnt) = i; 
                cnt = cnt + 1;  
                
            end
            
        end
        globalPop(del_list) = []; 
        del_list = []; 
        cnt = 1;
        
        
        % upgrade cost
        for k = 1:length(globalPop)
            globalPop(k).Cost = pdist([[robotPose(1),robotPose(2)];[globalPop(k).Pose(1,1),globalPop(k).Pose(1,2)]],'euclidean');
            
        end
        
        % 2. delete points which has distance costs less than 5
        for k=1:length(globalPop)
            if globalPop(k).Cost<5
                del_cost(cnt_) = k;
                cnt_ = cnt_ + 1;              
            end
        end 
        globalPop(del_cost) = [];
        del_cost = [];
        cnt_ = 1;

            
%             hold on
%             for k = 1:length(globalPop)
%                 figure(2)
%                 plot(globalPop(k).Pose(1,1),globalPop(k).Pose(1,2),'o')
%                 
%                 hold on
%             end
        
%         end
        
        
        
    end
    
    
%     obs_occ_check = getOccupancy(map_, [V1(1,1) V1(1,2)]);
    
    tmp_arr = [];
    alpha_inx = [];
    beta_inx = [];
    gamma_inx = [];
    alparr = [];
    straight_sen = [];
    obs_warning_1 = false;
    obs_warning_3 = false;
    obs_warning_2 = false;
    obs_warning_4 = false;
    obs_warning_5 = false;
    
    A1 = 2*a*rand(varSize) - a;
    C1 = 2*rand(varSize);
    A2 = 2*a*rand(varSize) - a;
    C2 = 2*rand(varSize);
    A3 = 2*a*rand(varSize) - a;
    C3 = 2*rand(varSize);
    if isempty(BestPosition)
            distance = 0;
    end
    
    for f = 1:length(group3_inx)
        straight_sen(f) = sensor_count(group3_inx(f));
        if straight_sen(f) < 3
           obs_warning_3 = true;
        end
    end
            
            for f = 1:length(group2_inx)
                straight_sen(f) = sensor_count(group2_inx(f));
                if straight_sen(f) < 2.8
                    obs_warning_2 = true;
                end
            end
            
            for f = 1:length(group4_inx)
                straight_sen(f) = sensor_count(group4_inx(f));
                if straight_sen(f) < 2.8
                    obs_warning_4 = true;
                end
            end
            
            for f = 1:length(group1_inx)
                straight_sen(f) = sensor_count(group1_inx(f));
                if straight_sen(f) < 2.5
                    obs_warning_1 = true;
                end
            end
            
            for f = 1:length(group5_inx)
                straight_sen(f) = sensor_count(group5_inx(f));
                if straight_sen(f) < 2.5
                    obs_warning_5 = true;
                end
            end
    
    
    
    
%     || group2V2_flag == true || group4V5_flag == true
    if j>1
        evalObstCounter(j) = 1;
        % GWO works when a robot need a help to change another diraction
        if group3V1_flag == true || group2V2_flag == true || group4V5_flag == true || group1V3_flag == true || group5V4_flag == true
          
            if isempty(BestPosition)
                distance = 0;
            else
                distance = pdist([[BestPosition(1,1),BestPosition(1,2)];[x,y]],'euclidean');
            end
            
            if steps > 15
               distance = 2;
            end
            
            
            
           
            %|| obs_warning_3 || obs_warning_4 || obs_warning_2

            if distance<5 
                steps = 0;
                
                
                % obstacle detected
                % the closest ones (alpha group)
                % closer ones (beta group)
                % close ones (gamma group)
                for k=1:length(globalPop)
                    tmp_arr(k) = globalPop(k).Cost;
                end

                [sorted_x, index] = sort(tmp_arr(1,:),'ascend');  
                
                if index <= 10
                    alpha_inx = index(1:length(index));
                else
                    alpha_inx = index(1:10);
                    if index<20
                        beta_inx = index(11:length(index));
                    else
                        beta_inx = index(11:20);
                        if index<30
                            gamma_inx = index(21:length(index));
                        else
                        gamma_inx = index(21:30);
                        end
                    end 

                end
                
                
                
                             
                
                

                for i = 1:length(alpha_inx)
                   alpha(i).Pose = globalPop(alpha_inx(i)).Pose; 
                   alpha(i).Cost = globalPop(alpha_inx(i)).Cost;
                   alpha(i).Ind = alpha_inx(i);
                end


                for i = 1:length(beta_inx)
                   beta(i).Pose = globalPop(beta_inx(i)).Pose; 
                   beta(i).Cost = globalPop(beta_inx(i)).Cost;
                   beta(i).Ind = beta_inx(i);
                end

                for i = 1:length(gamma_inx)
                   gamma(i).Pose = globalPop(gamma_inx(i)).Pose; 
                   gamma(i).Cost = globalPop(gamma_inx(i)).Cost;
                   gamma(i).Ind = gamma_inx(i);
                end

                for i = 1:length(alpha)
                    alparr(i) = alpha(i).Cost;
                end
                MeanAlpha = mean([alpha.Cost]);
                [alpclval, alpclindex] = min(abs(alparr - MeanAlpha));

                for i = 1:length(beta)
                    betaarr(i) = beta(i).Cost;
                end
                MeanBeta = mean([beta.Cost]);
                [betaclval, betaclindex] = min(abs(betaarr - MeanBeta));

                for i = 1:length(gamma)
                    gammaarr(i) = gamma(i).Cost;
                end
                MeanGamma = mean([gamma.Cost]);
                [gammaclval, gammaclindex] = min(abs(gammaarr - MeanGamma));

                Dalpha = abs(C1.*alpha(alpclindex).Pose - [x,y]);
                X1 = alpha(alpclindex).Pose - A1.*Dalpha;
                Dbeta = abs(C2.*beta(betaclindex).Pose - [x,y]);
                X2 = beta(betaclindex).Pose - A2.*Dbeta;
                Dgamma = abs(C3.*gamma(gammaclindex).Pose - [x,y]);
                X3 = gamma(gammaclindex).Pose - A3.*Dgamma;

                BestPosition = abs((X1 + X2 + X3)/3);
                point = [BestPosition(1,1), BestPosition(1,2)];
                
                if BestPosition(1,1) > (mapsize+10)
                    BestPosition(1,1) = mapsize+9;
                end
                
                if BestPosition(1,2) > mapsize
                    BestPosition(1,2) = mapsize-1;
                end
                
                if point(1,1)>(mapsize+10)
                    point(1,1) = mapsize+9;
                end
                if point(1,2)>mapsize
                    point(1,2) = mapsize-1;
                end
                 hold on 
                 gwopointcounter = gwopointcounter + 1;
                 str = sprintf('G%i', gwopointcounter);
                 text(point(1,1), point(1,2), str, 'Color','black','FontSize',7);
                 
                 
                 
            
            else
                
                
                
                if obs_warning_1 == true || obs_warning_2 == true || obs_warning_3 == true || obs_warning_4 == true || obs_warning_5 == true
                     
                    % if escape point is not selected yet, steps == 0
                    goback_ = 0;
                    if steps == 0
                        evalObstCounter(j) = 0;
                        % 1 - obstacle, 0 - free
                        obs_warn = [obs_warning_3, obs_warning_2, obs_warning_1, obs_warning_5, obs_warning_4];
                        % find free element of array (find 0)
                        % X has elements of free paths
                        
                        % occVn - it should be known and free space,
                        % because Vn lays in sensor range; so if - 0
                        % (known), choose the point
                        [V1, V2, V3, V4, V5] = avoidObstacle(robotPose);
                        
                        
                       
                        
                        intsectionPtsV1 = rayIntersection(map,[V1(1,1), V1(1,2), robotPose(3)],angleObs,maxrangeObs);
                        intsectionPtsV2 = rayIntersection(map,[V2(1,1), V2(1,2), robotPose(3)],angleObs,maxrangeObs);
                        intsectionPtsV3 = rayIntersection(map,[V3(1,1), V3(1,2), robotPose(3)],angleObs,maxrangeObs);
                        intsectionPtsV4 = rayIntersection(map,[V4(1,1), V4(1,2), robotPose(3)],angleObs,maxrangeObs);
                        intsectionPtsV5 = rayIntersection(map,[V5(1,1), V5(1,2), robotPose(3)],angleObs,maxrangeObs);
                        
                        cntIntsV1 = 0;
                        cntIntsV2 = 0;
                        cntIntsV3 = 0;
                        cntIntsV4 = 0;
                        cntIntsV5 = 0;
                        
                        for h = 1:8
                            if isnan(intsectionPtsV1(h))
                                cntIntsV1 = cntIntsV1 + 1;
                            end
                            if isnan(intsectionPtsV2(h))
                                cntIntsV2 = cntIntsV2 + 1;
                            end
                            if isnan(intsectionPtsV3(h))
                                cntIntsV3 = cntIntsV3 + 1;
                            end
                            if isnan(intsectionPtsV4(h))
                                cntIntsV4 = cntIntsV4 + 1;
                            end
                            if isnan(intsectionPtsV5(h))
                                cntIntsV5 = cntIntsV5 + 1;
                            end
                            
                        end
                        
                        if V1(1,2) > 0
                            if cntIntsV1 == 8 || cntIntsV1 == 7
                                occV1 = checkOccupancy(map, [V1(1,1),V1(1,2)]);
                            else
                                occV1 = 1;
                            end
                        else
                            occV1 = 1;
                            
                        end
                        
                        if V2(1,2) > 0
                            if cntIntsV2 == 8 || cntIntsV2 == 7
                                occV2 = checkOccupancy(map, [V2(1,1),V2(1,2)]);
                            else
                                occV2 = 1;
                            end                            
                        else
                            occV2 = 1;
                            
                        end
                        
                        if V3(1,2) > 0
                            if cntIntsV3 == 8 || cntIntsV3 == 7
                                occV3 = checkOccupancy(map, [V3(1,1),V3(1,2)]);
                            else
                                occV3 = 1;
                            end
                            
                        else
                            occV3 = 1;
                            
                        end
                        
                        
                        
                        if V4(1,2) > 0 
                            if cntIntsV4 == 8 || cntIntsV4 == 7
                                occV4 = checkOccupancy(map, [V4(1,1),V4(1,2)]);
                            else
                                occV4 = 1;
                            end
                        else
                            occV4 = 1;
                            
                        end
                        
                        
                        if V5(1,2) > 0 
                            if cntIntsV5 == 8 || cntIntsV5 == 7
                                occV5 = checkOccupancy(map, [V5(1,1),V5(1,2)]);
                            else
                                occV5 = 1;
                            end
                            
                        else
                            occV5 = 1;
                            
                        end
                        
                        
                       
                        
                        
                        
                        
                        
                        obs_warn = [occV1, occV2, occV3, occV4, occV5];
                        
                        X = find(~obs_warn);
                        if occV1 ~= 0
                            if any(X(:)==1)
                                X(X(1,:) == 1) = [];
                            end
                        end
                            
                        if occV2 ~= 0
                            if any(X(:)==2)
                                X(X(1,:) == 2) = [];
                            end
                        end
                        
                        if occV3 ~= 0
                            if any(X(:)==3)
                                X(X(1,:) == 3) = [];
                            end
                        end
                        
                        if occV4 ~= 0
                            if any(X(:)==4)
                                X(X(1,:) == 4) = [];
                            end
                        end
                        
                        if occV5 ~= 0
                            if any(X(:)== 5)
                                X(X(1,:) == 5) = [];
                            end
                        end
                        
                        
                        
                        if isempty(X)                           
                            [Vback] = goback(robotPose);
                            point_obs = [V1_(1,1),V1_(1,2)];
                            disp("go back");
                            goback_ = goback_ + 1;
                        else
                            % random choose free path
        %                     Xsize = length(X);
                            Vrandom = randi(length(X),1);
                            Vrandom_value = X(Vrandom);
                            if Vrandom_value == 1
                                point_obs = [V1(1,1),V1(1,2)];
                            end
                            if Vrandom_value == 2
                                point_obs = [V2(1,1),V2(1,2)];
                            end
                            if Vrandom_value == 3
                                point_obs = [V3(1,1),V3(1,2)];
                            end
                            if Vrandom_value == 4
                                point_obs = [V4(1,1),V4(1,2)];
                            end
                            if Vrandom_value == 5
                                point_obs = [V5(1,1),V5(1,2)];
                            end

                        end
                        steps = steps + 1;
%                         distance_point = pdist([[point(1,1),point(1,2)];[x,y]],'euclidean');
%                         hold on 
%                         plot(point_obs(1,1), point_obs(1,2), 'Marker', '*');
                        point = [point_obs(1,1), point_obs(1,2)];
                     % if escape point was selected already, then increase counter    
                    else
                        evalObstCounter(j) = 0;
                        distance_point = pdist([[point_obs(1,1),point_obs(1,2)];[x,y]],'euclidean');
                        if distance_point < 0.1
                            steps = 0;
                            point = [BestPosition(1,1), BestPosition(1,2)];
                        else
                            
                            
                            steps = steps + 1;
                            point = [point_obs(1,1), point_obs(1,2)];
                            plot(point_obs(1,1), point_obs(1,2), 'Marker', 'o');
                        end
                    end
%                     hold on
%                     plot(V1(1,1),V1(1,2),'*');
%                     plot(V2(1,1),V2(1,2),'*');
%                     plot(V3(1,1),V3(1,2),'*');
%                     plot(V4(1,1),V4(1,2),'*');
%                     plot(V5(1,1),V5(1,2),'*');
                    
                    
                    
                else
                    point = [BestPosition(1,1), BestPosition(1,2)];
                    steps = 0;
                end                
            end
        else
            % if there is no any obstacles, but GWO was computed in previous steps, 
            % so let's a robot goes the bestPosition; another words: let a
            % robot continue the way to global point
            point = [BestPosition(1,1), BestPosition(1,2)];
            
        end
    else
        % if iteration = 1
        point = [V1_(1,1),V1_(1,2)];
        evalObstCounter(j) = 1;
    end


%     
    % change waypoint if there is obstacle
    
%     [endPts,midPts] = raycast(map,robotPose, 5, robot.LaserSensor.AngleSweep(26));
    
    
    
    
    insertRay(map, robotPose, modScan, robot.LaserSensor.MaxRange);
    % v - velocity, w - angular velocity
    [v, w] = controller(robotPose);
    velMsg.Linear.X = v;
    velMsg.Angular.Z = w;
    send(velPub, velMsg);
    disp(robotPose);
    controller.Waypoints = point;
    poseArray = [poseArray; robotPose];
    if j>1
        poseArrDist = poseArrDist + pdist([poseArray(j,1),poseArray(j,2);poseArray(j-1,1),poseArray(j-1,2)],'euclidean');
    end
    
    
    % Wait for control rate to ensure 10 Hz rate.
    waitfor(controlRate);
    a = a - adamp;
    if a < 1
        a = 2;
    end
    
    if j == 2200
        map_final_counter = 0;
        mapOccTotal = [];
        for n = 1:(mapsize+10)
            for m=1:mapsize                
                mapOccTotal(n,m) = checkOccupancy(map,[n m]);                
            end
        end 
        
        for n = 1:(mapsize+10)
            for m=1:mapsize
                if mapOccInitial(n,m) ~= 8
                    if mapOccTotal(n,m) == 0
                        map_final_counter = map_final_counter + 1;
                    end
                end
            end
        end 
        procent = 100 - ((map_counter - map_final_counter)/map_counter)*100;
%         procent_ = 100 - procent;
    end
    
%     
    if j == 1
        map_counter = 0;
        mapOccInitial = 0;
        obs_value_map = 8;
        
       for n = 1:(mapsize+10)
            for m=1:mapsize              
                mapOccInitial(n,m) = checkOccupancy(map,[n m]);                 
            end
       end 
       mapOccInitial(1:30,1) = obs_value_map;
       mapOccInitial(1:30,19) = obs_value_map;
       mapOccInitial(1:30,20) = obs_value_map;
       mapOccInitial(1,1:20) = obs_value_map;
       mapOccInitial(30,1:20) = obs_value_map;
       mapOccInitial(29,1:20) = obs_value_map;
       mapOccInitial(8:10,1:6) = obs_value_map;
       mapOccInitial(8:10,14:20) = obs_value_map;
       mapOccInitial(19:25,4:5) = obs_value_map;
       mapOccInitial(19:30,11:12) = obs_value_map;
       
       for n = 1:(mapsize+10)
            for m=1:mapsize              
                if mapOccInitial(n,m) ~= 8
                    map_counter = map_counter + 1;
                end
            end
       end 
       
       

    end
    
end
figure(3)
subplot(2,1,1);
plot(exploredness, 'LineWidth', 0.8, 'color', 'black')
ylim([-0.5 1.5])
xlim([1 2300])
title('Uncertainties Search Decisions');

subplot(2,1,2);
plot(evalObstCounter, 'LineWidth', 0.8, 'color', 'black')
ylim([-0.5 1.5])
xlim([1 2300])
title('Obstacle Avoidance Decisions');

clear('node')
% rosshutdown

