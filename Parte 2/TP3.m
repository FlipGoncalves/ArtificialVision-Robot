%%% Add tools to path
addpath 'fanuctoolbox'

%%% Clear variables
clc
close all
clear

% Robot camera index
wbcm = 2;

%%% Connect to robot
% Define communication variables
robCOMM.port = 4900;
robCOMM.ip = '192.168.0.229';

% Try to connect to the manipulator
while 1
    try
        % Create the TCP IP object
        robCOMM.handle = tcpclient(robCOMM.ip, robCOMM.port);
        % 1 sec timeout
        set(robCOMM.handle, 'Timeout', 1);
        break
    catch
        fprintf("Can't connect to the server...\nReloading...\n")
    end
end

% Open the object
fopen(robCOMM.handle);

%%% Calibrate camera
cameraParams = calibrateCamera(1, wbcm);   % 1 for windows / 0 for linux 

input("Press <ENTER> to START")

%%% Move robot to initial position
% Turn off vacuum
Gripper_Open(robCOMM, 2);

% Get actual position
Pos = Get_Cart_Abs(robCOMM);

% Move robot to initial position
P = [400, 0, 220];
newP = [P(1)-Pos(1,1), P(2)-Pos(1,2), P(3)-Pos(1,3)];
Mov_Cart_Inc(robCOMM,newP(1),newP(2),newP(3),0,0,1,1)
input("Press <ENTER> to CONTINUE")

%%% Find objects
cam = webcam(wbcm);

image = snapshot(cam);
[im, newOrigin] = undistortImage(image, cameraParams, 'OutputView', 'full');
[imagePoints, boardSize] = detectCheckerboardPoints(im);
[R, t] = extrinsics(imagePoints, cameraParams.WorldPoints, cameraParams);

% convert rgb image to hsv
[H,S,V] = rgb2hsv(im);

% mask the objects/plane by color
greenObjectsMask = (H > 0.2 & H < 0.5 & S > 0.4 & S < 1.0 & V > 0.3);
redObjectsMask   = (H > 0.0 & H < 0.12 & S > 0.4 & S < 1.0 & V > 0.1);

% Label connected components in the binary mask
labeledRedImage = bwlabel(redObjectsMask);
labeledGreenImage = bwlabel(greenObjectsMask);

% Calculate region properties, including centroids
Redstats = regionprops(labeledRedImage, 'Centroid', 'Area');
Greenstats = regionprops(labeledGreenImage, 'Centroid', 'Area');

% Get centroids of the regions with largest area
num_red_objects = 1;
num_green_objects = 1;
Points = [];
End_Object = [];

subplot(1,2,1);
imshow(image)
hold on;

% Get red objects
[~, ind] = maxk([Redstats.Area], num_red_objects+1);
Red_Plane = Redstats(ind(1)).Centroid;
plot(Red_Plane(1), Red_Plane(2), "r*")
for i=2:size(ind,2)
    P = Redstats(ind(i)).Centroid;
    Points = [Points; P];
    End_Object = [End_Object; Red_Plane];
    plot(P(1), P(2), "r*")
end

subplot(1,2,2);
imshow(image)
hold on;

% Get green objects
[~, ind] = maxk([Greenstats.Area], num_green_objects+1);
Green_Plane = Greenstats(ind(1)).Centroid;  
plot(Green_Plane(1), Green_Plane(2), "r*")
for i=2:size(ind,2)
    P = Greenstats(ind(i)).Centroid;
    Points = [Points; P];
    End_Object = [End_Object; Green_Plane];
    plot(P(1), P(2), "r*")
end

%%% Move the robot
% Move to each object and place it in the correct plane 
for i=1:size(Points, 1)

    input("Press <ENTER> to CONTINUE")

    % Get actual position
    Pos = Get_Cart_Abs(robCOMM);

    % R T N -> Transform of N in R
    T = [ 0 1 0  327
          1 0 0 -459
          0 0 0 -206
          0 0 0  0   ];

    % Convert object camera point to real world point
    object = [pointsToWorld(cameraParams, R, t, [Points(i,1), Points(i,2)]) 1 1];
    object_robo = T * object';
    P = [object_robo(1)-Pos(1,1), object_robo(2)-Pos(1,2), 164-Pos(1,3)];
    
    % Robot goes to the object and down to pick up the object
    Mov_Cart_Inc(robCOMM,P(1),P(2),P(3),0,0,1,1)
    pause(0.5);
    Mov_Cart_Inc(robCOMM,0,0,-333,0,0,1,1)
    pause(0.5);

    % Update robot position
    object_robo = [object_robo(1), object_robo(2), 164-331];

    % Turn on vacuum
    Gripper_Close(robCOMM, 2);

    % Robot goes up
    Mov_Cart_Inc(robCOMM,0,0,160,0,0,1,1)
    pause(0.5);

    % Update robot position
    object_robo = [object_robo(1), object_robo(2), object_robo(3)+160];

    % Convert plane camera point to real world point
    object = [pointsToWorld(cameraParams, R, t, [End_Object(i,1), End_Object(i,2)]) 1 1];
    Plane = T * object';
    P = [Plane(1)-object_robo(1), Plane(2)-object_robo(2)];

    % Robot goes to the plane
    Mov_Cart_Inc(robCOMM,P(1),P(2),0,0,0,1,1)
    pause(0.5);

    % Update robot position
    object_robo = [Plane(1), Plane(2), object_robo(3)];

    % Robot goes down
    Mov_Cart_Inc(robCOMM,0,0,-160+60*(i/2),0,0,1,1)
    pause(0.5);

    % Update robot position
    object_robo = [object_robo(1), object_robo(2), object_robo(3)-160];

    % Turn off vacuum
    Gripper_Open(robCOMM, 2);
end

%%% End program
input("Press <ENTER> to CONTINUE")

% Get actual position
Pos = Get_Cart_Abs(robCOMM);

% Move robot to initial position
P = [400, 0, 220];
newP = [P(1)-Pos(1,1), P(2)-Pos(1,2), P(3)-Pos(1,3)];
Mov_Cart_Inc(robCOMM,newP(1),newP(2),newP(3),0,0,1,1)

% End
fprintf("DONE\n");