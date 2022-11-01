clc;
clear;
close all;
rosinit('192.168.0.8')
tbot = turtlebot('192.168.0.8');

% create publisher for robot's velocity
robot = rospublisher('/mobile_base/commands/velocity');
% create a message for the topic
velmsg = rosmessage(robot);

% TurtleBot uses the Kinect data to simulate a laser scan that is published on the /scan topic
laser = rossubscriber('/scan','BufferSize', 5);
% scan = receive(laser,3);

spinVelocity = 0.8;
forwardVelocity = 0.5;    % Linear velocity (m/s)
backwardVelocity = -0.02; % Linear velocity (reverse) (m/s)
distThresholdLocalPoint = 0.5;  % Distance threshold (m) for turning
distThresholdGlobalPoint = 4;

maxRange = 10;

% odom data is needed for knowing current robot position
odom = rossubscriber('/odom', 'BufferSize', 25);
point = [1 0];
controller = robotics.PurePursuit;
controller.LookaheadDistance = 0.1;
controller.DesiredLinearVelocity = 0.2;
controller.MaxAngularVelocity = 0.2;
controller.Waypoints = [0 0; point];


maxSensorRange = 4;

controlRate = robotics.Rate(10);

pursuitGPcnts = 1;

j = 0;

%%
%Parameters for GWO & globalPop points
globalPop.Pose = [];
globalPop.Cost = [];
globalPop.Occ = [];
nIternCnt = 1;
del_list = [];
del_ident = [];
del_cost = [];
save = [];
cnt_=1;
cnt = 1;
ident = 1;
BestPosition = [];
gwopointcounter = 1;
time = 2000;
a = 2;
adamp = a/500;
varSize = [1 2];
localObs = false;
strGWO.Number = [];
strGWO.Point = [];
strGWO.Number = gwopointcounter;
strGWO.Point = [point(1,1),point(1,2)];
obs_warn_inx = [];
pointObsCnt = 0;
V = [];
angleObs = [0, pi/4, pi/2, 3*pi/4, pi, 5*pi/4, 3*pi/2, 7*pi/4];
maxrangeObs = 0.5;
poses = [0, 0, 0];
scans = {};
robotPose = [];
countPts = 1;
points = [];
dist_point = 0;
sv = 1;
while controlRate.TotalElapsedTime < time
  laserScan = receive(laser);
  odomdata = receive(odom);
%   tform = getTransform(tbot,'base_link','camera_link');
  scan = lidarScan(laserScan);
  ranges = scan.Ranges;
  rangesNotNan = scan.Ranges;
  rangesNotNan(isnan(rangesNotNan)) = maxSensorRange;
  rangesNotNan(isinf(rangesNotNan)) = maxSensorRange;
%   ranges(isnan(ranges)) = laserScan.RangeMax;
  angles = scan.Angles;
 
  % Create a lidarScan object from the ranges and angles
  modScan = lidarScan(ranges,angles);
  scans(nIternCnt) = {modScan};
  nIternCnt = nIternCnt + 1;
  
  pose = odomdata.Pose.Pose;
  x_odom = pose.Position.X;
  y_odom = pose.Position.Y;
  z_odom = pose.Position.Z;
  quat = pose.Orientation;
  angles = quat2eul([quat.W quat.X quat.Y quat.Z],'ZYX');
  theta = rad2deg(angles(1));
  position = [pose.Position.X, pose.Position.Y];
  pose = [position,angles(1)];
  robotPose = [robotPose; pose];
  poses = [poses; pose];
  V = [pose(1,1)+2,pose(1,2),pose(1,3)]; 
  % endpoints of laser
  % data array has only x y point of close obstacle / no nan data
  data = readCartesian(laserScan);
  % Compute distance of the closest obstacle
  x_laser = data(:,1);
  y_laser = data(:,2);
  dist = sqrt(x_laser.^2 + y_laser.^2);
  minDist = min(dist); 
  
  % Command robot action
  
  
  if nIternCnt > 2
      countPts = length(globalPop);
      if countPts == 0
          countPts = 1;
      end
        for i = 1:length(laserScan.Ranges)
             sensor_count = laserScan.Ranges;
             angles_count = laserScan.readScanAngles;
             if isinf(sensor_count(i))
                     [endPts] = raycast(map,pose, rangesNotNan(i), angles_count(i));
                 
                 if ~isempty(endPts)
%                      if getOccMidPflag == false
                        temp_value = grid2world(map, endPts);
                        globalPop(countPts).Pose = [temp_value(1,1), temp_value(1,2)];   
                        % the costs are only for new frontier points  
                        globalPop(countPts).Cost = pdist([[pose(1),pose(2)];[temp_value(1,1), temp_value(1,2)]],'euclidean');
                        countPts = countPts + 1;
%                      end
                 end
             end
        end
            
        %      filters; remove the frontier points
       if length(globalPop) > 1
            for i = 1:length(globalPop)
            
                occ = getOccupancy(map,[globalPop(i).Pose(1,1), globalPop(i).Pose(1,2)]);
                globalPop(i).Occ = occ;      
               if occ<0.5 || occ>0.7
                    del_list(cnt) = i; 
                    cnt = cnt + 1;           
               end
            end
       
            globalPop(del_list) = []; 
            del_list = []; 
            cnt = 1; 
            
           for i = 1:length(globalPop)-1
              if globalPop(i).Pose == globalPop(i+1).Pose
                  if globalPop(i).Cost == globalPop(i+1).Cost 
                      del_ident(ident) = i;
                      ident = ident + 1;
                  end
              end
           end
           
           globalPop(del_ident) = []; 
           del_ident = []; 
           ident = 1;
           
        
            % upgrade cost
            for k = 1:length(globalPop)
                globalPop(k).Cost = pdist([[pose(1),pose(2)];[globalPop(k).Pose(1,1),globalPop(k).Pose(1,2)]],'euclidean');       
            end
            
%             for k=1:length(globalPop)
%                 [endpoints, midpoints] = raycast(map, [pose(1),pose(2)],globalPop(k).Pose);
%                 temp_value_ = grid2world(map, midpoints);
%                 for i=1:length(midpoints)
%                     occMid = getOccupancy(map,temp_value_(i,:));
%                     if occMid > 0.55
%                         save(sv) = k;
%                         sv = sv + 1;
%                         break;
%                     end
%                 end
%                 
%             end
%             
            
%            globalPop(save) = []; 
%            save = []; 
%            sv = 1;

%             glblth = length(globalPop);
%             delglb = 0;
%             if glblth > 200
%                 delglb = glblth - 200;
%                 
%                 globalPop = [globalPop(delglb:end)];
%             end

               if nIternCnt > 150
                   for k=1:length(globalPop)
                       if globalPop(k).Cost < 3.5
                           save(sv) = k;
                           sv = sv + 1;
                       end
                   end
               end
               
               globalPop(save) = []; 
               save = []; 
               sv = 1;
               
            
%             for k=1:length(globalPop)
%                 if globalPop(k).Cost < maxSensorRange
%                     del_cost(cnt_) = k;
%                     cnt_ = cnt_ + 1;              
%                 end
%             end 
%             globalPop(del_cost) = [];
%             del_cost = [];
%             cnt_ = 1;
            
            
      end
  end
  localObs = false;
  if nIternCnt > 4
                  strGWOlgth = length(strGWO);
                  minDistGL = pdist([[pose(1),pose(2)];[strGWO(strGWOlgth).Point(1,1),strGWO(strGWOlgth).Point(1,2)]],'euclidean');
                  if minDist < distThresholdLocalPoint
                      localObs = true;
                      disp("Select Local Point");
                      minDistGL = 0;
                  end
                  
                  if minDistGL < distThresholdGlobalPoint
%                       if minDist < distThresholdLocalPoint
%                           localObs = true;
%                           disp("Select Local Point");

%                           point = point_obs;

%                       else
                         pursuitGPcnts = 0; 
%                          localObs = false; 
                         disp("Select Global Point");
                         tmp_arr = [];
                         alpha_inx = [];
                         beta_inx = [];
                         gamma_inx = [];
                         alparr = [];

                         A1 = 2*a*rand(varSize) - a;
                         C1 = 2*rand(varSize);
                         A2 = 2*a*rand(varSize) - a;
                         C2 = 2*rand(varSize);
                         A3 = 2*a*rand(varSize) - a;
                         C3 = 2*rand(varSize);


                         for k=1:length(globalPop)
                                tmp_arr(k) = globalPop(k).Cost;
                         end

                         [sorted_x, index] = sort(tmp_arr(1,:),'ascend');  
                         
                         if index < 10
                             alpha_inx = index(1:length(index));
                         else
                             alpha_inx = index(1:10);
                             
                             if index < 20
                                beta_inx = index(11:length(index));
                             else
                                beta_inx = index(11:20);
                                if index < 30
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
                         
                         if exist('beta', 'var') == 0
                             beta(1).Pose = 0;
                             beta(1).Cost = 0;
                             beta(1).Ind = 1;
                         end
                         if isempty(beta)
                             beta(1).Pose = 0;
                             beta(1).Cost = 0;
                             beta(1).Ind = 1;
                         end

                         for i = 1:length(beta)
                                 betaarr(i) = beta(i).Cost;
                         end
                         MeanBeta = mean([beta.Cost]);
                         [betaclval, betaclindex] = min(abs(betaarr - MeanBeta));
                         
                         if exist('gamma', 'var') == 0
                             gamma(1).Pose = 0;
                             gamma(1).Cost = 0;
                             gamma(1).Ind = 1;
                         end
                         if isempty(gamma)
                             gamma(1).Pose = 0;
                             gamma(1).Cost = 0;
                             gamma(1).Ind = 1;
                         end
                         for i = 1:length(gamma)
                             gammaarr(i) = gamma(i).Cost;
                         end
                         MeanGamma = mean([gamma.Cost]);
                         [gammaclval, gammaclindex] = min(abs(gammaarr - MeanGamma));

                         Dalpha = C1.*alpha(alpclindex).Pose - [pose(1,1),pose(1,2)];
                         X1 = alpha(alpclindex).Pose - A1.*Dalpha;
                         Dbeta = C2.*beta(betaclindex).Pose - [pose(1,1),pose(1,2)];
                         X2 = beta(betaclindex).Pose - A2.*Dbeta;
                         Dgamma = C3.*gamma(gammaclindex).Pose - [pose(1,1),pose(1,2)];
                         X3 = gamma(gammaclindex).Pose - A3.*Dgamma;

                         BestPosition = (X1 + X2 + X3)/3;
                         alpha = [];
                         beta = [];
                         gamma = [];
                         point = [BestPosition(1,1), BestPosition(1,2)];

                         gwopointcounter = gwopointcounter + 1;
                         strGWO(gwopointcounter).Number = gwopointcounter;
                         strGWO(gwopointcounter).Point = [point(1,1),point(1,2)];

                         a = a - adamp;
%                          if a < 1
%                              a = 2;
%                          end 
%                       end 

                  else
%                       point = [BestPosition(1,1), BestPosition(1,2)];
%                       localObs = false;
                      if isempty(BestPosition)
                          point = [V(1,1),V(1,2)];
                      else 
                          point = [BestPosition(1,1), BestPosition(1,2)];
                      end
%                       pursuitGPcnts = pursuitGPcnts + 1;
                  end
  else
      localObs = false;
      point = [V(1,1),V(1,2)];
  end   
  
  if localObs == true
     velmsg.Linear.X = backwardVelocity;
     velmsg.Angular.Z = spinVelocity;
     send(robot, velmsg);
     disp(pose);
  else
     controller.Waypoints = point; 
     [v, w] = controller(pose);
     velmsg.Linear.X = v;
     velmsg.Angular.Z = w;
     disp("v = " + v);
     disp("w = " + w);
%     
     send(robot, velmsg);
     disp("Current position: x=" + pose(1) + ", y=" + pose(2) + ", z=" + pose(3));
     disp("GP point: " + point(1) + " " + point(2));
     
  end
  
  map = buildMap(scans,robotPose,10, 10);
  show(map)
  hold on
%   poses(1,:) = [];
  
  plot(pose(1,1), pose(1,2), 'marker','o', 'MarkerSize', 3.4, 'color', 'r', 'MarkerFaceColor','red');    
%   globalPop_ = globalPop;
%   globalPop = [];
  j = j + 1;
  
%   % plot orientation line
%   distAheadx = robotPose(1) + cos(robotPose(3))*1.5;
%   distAheady = robotPose(2) + sin(robotPose(3))*1.5;
%   drawnow
%   clearpoints()
%   plot([robotPose(1,1) distAheadx], [robotPose(1,2) distAheady], 'color', 'red', 'LineWidth', 1);
  hold on
  for i = 1:length(strGWO)
      str = sprintf('G%i', strGWO(i).Number);
      text(strGWO(i).Point(1,1), strGWO(i).Point(1,2), str, 'Color','black','FontSize',8, 'fontweight', 'bold');
      hold on
  end
%     waitfor(controlRate)
end
poseArray = [0 0 0];
poseArrDist = 0;
poseArray = [poseArray; robotPose];
for j = 2:length(poseArray)
    poseArrDist = poseArrDist + pdist([poseArray(j,1),poseArray(j,2);poseArray(j-1,1),poseArray(j-1,2)],'euclidean');
end

% hold on
% text(point(1,1), point(1,2), '*', 'color', 'red', 'FontSize', 7);

%  hold on
%             for k = 1:length(globalPop)
%                 plot(globalPop(k).Pose(1,1),globalPop(k).Pose(1,2),'o')
%                 
%                 hold on
%             end

hold on
plot(robotPose(:,1),robotPose(:,2),'LineWidth',5, 'Color','red')

% 
tbot.resetOdometry();
clear('node')
rosshutdown