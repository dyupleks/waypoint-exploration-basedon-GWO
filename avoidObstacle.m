function [V1, V2, V3, V4, V5] = avoidObstacle(robotPose)

x = robotPose(1,1);
y = robotPose(1,2);
th = robotPose(1,3);
distanceLookAhead = 2;

thV1 = 0;
thV2 = -pi/4;
thV3 = -pi/2;
thV4 = pi/2;
thV5 = pi/4;

V1(1,1) = (x + cos(th + thV1)*distanceLookAhead) ;
V1(1,2) = (y + sin(th + thV1)*distanceLookAhead) ;

V2(1,1) = (x + cos(th + thV2)*distanceLookAhead) ;
V2(1,2) = (y + sin(th + thV2)*distanceLookAhead) ;

V3(1,1) = (x + cos(th + thV3)*distanceLookAhead) ;
V3(1,2) = (y + sin(th + thV3)*distanceLookAhead) ;

V4(1,1) = (x + cos(th + thV4)*distanceLookAhead) ;
V4(1,2) = (y + sin(th + thV4)*distanceLookAhead) ;

V5(1,1) = (x + cos(th + thV5)*distanceLookAhead) ;
V5(1,2) = (y + sin(th + thV5)*distanceLookAhead) ;


if V1(1,2) < 1
    V1(1,2) = 1;
end
                        
if V2(1,2) < 1
    V2(1,2) = 1;
end
                        
if V3(1,2) < 1
   V3(1,2) = 1;
end
                        
                        
                        
if V4(1,2) < 1 
   V4(1,2) = 1;
end
                        
                        
if V5(1,2) < 1 
    V5(1,2) = 1;
end
                        
if V1(1,2) > 19
   V1(1,2) = 19;
end
                        
if V2(1,2) > 19
   V2(1,2) = 19;
end
                        
if V3(1,2) > 19
   V3(1,2) = 19;
end
                      
if V4(1,2) > 19 
    V4(1,2) = 19;
end
                        
                        
if V5(1,2) > 19 
   V5(1,2) = 19;
end


if V1(1,1) > 29
   V1(1,1) = 29;
end
                        
if V2(1,1) > 29
   V2(1,1) = 29;
end
                        
if V3(1,1) > 29
   V3(1,1) = 29;
end
                      
if V4(1,1) > 29 
    V4(1,1) = 29;
end
                                              
if V5(1,1) > 29 
   V5(1,1) = 29;
end


if V1(1,1) < 1
   V1(1,1) = 1;
end
                        
if V2(1,1) < 1
   V2(1,1) = 1;
end
                        
if V3(1,1) < 1
   V3(1,1) = 1;
end
                      
if V4(1,1) < 1 
    V4(1,1) = 1;
end
                                              
if V5(1,1) < 1 
   V5(1,1) = 1;
end

