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
imshow(image);
[im, newOrigin] = undistortImage(image, cameraParams, 'OutputView', 'full');
[imagePoints, boardSize] = detectCheckerboardPoints(im);
[R, t] = extrinsics(imagePoints, cameraParams.WorldPoints, cameraParams);

% convert rgb image to hsv
[H,S,V] = rgb2hsv(im);

% mask the objects/plane by color
greenObjectsMask = (H >= 0.2 & H <= 0.5 & S >= 0.2 & S <= 0.8 & V >= 0.2);
redObjectsMask = (H <= 0.2 & S >= 0.8 & V >= 0.8);

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

% Get Red Plane
[~, ind] = max([Redstats.Area]);
Red_Plane = Redstats(ind).Centroid;
Redstats(ind) = [];

% Get green plane
[~, ind] = max([Greenstats.Area]);
Green_Plane = Greenstats(ind).Centroid;
Greenstats(ind) = [];

% Get red objects
for i=1:num_red_objects
    [~, ind] = max([Redstats.Area]);
    Points = [Points; Redstats(ind).Centroid];
    End_Object = [End_Object; Red_Plane];
    Redstats(ind) = [];
end
% Get green objects
for i=1:num_green_objects
    [~, ind] = max([Greenstats.Area]);
    Points = [Points; Greenstats(ind).Centroid];
    End_Object = [End_Object; Green_Plane];
    Greenstats(ind) = [];
end

%%% Move the robot
% Move to each object and place it in the correct plane 
for i=1:size(Points, 1)
    input("Press <ENTER> to CONTINUE")

    % Get actual position
    Pos = Get_Cart_Abs(robCOMM);

    % R T N -> Transform of N in R
    T = [ 0 1 0  325
          1 0 0 -440
          0 0 0 -160
          0 0 0  0   ];

    % Convert object camera point to real world point
    object = [pointsToWorld(cameraParams, R, t, [Points(i,1), Points(i,2)]) 1 1];
    object_robo = T * object';
    P = [object_robo(1)-Pos(1,1), object_robo(2)-Pos(1,2), 164-Pos(1,3)];

    % Robot goes down to pick up object
    Mov_Cart_Inc(robCOMM,P(1),P(2),P(3),0,0,1,1)
    pause(0.5);
    Mov_Cart_Inc(robCOMM,0,0,-331,0,0,1,1)

    % Update robot position
    object_robo = [object_robo(1), object_robo(2), 164-331];

    % Turn on vacuum
    Gripper_Close(robCOMM, 2);

    % Robot goes up
    Mov_Cart_Inc(robCOMM,0,0,160,0,0,1,1)

    % Update robot position
    object_robo = [object_robo(1), object_robo(2), object_robo(3)+160];

    % Convert plane camera point to real world point
    object = [pointsToWorld(cameraParams, R, t, [End_Object(i,1), End_Object(i,2)]) 1 1];
    Plane = T * object';
    P = [Plane(1)-object_robo(1), Plane(2)-object_robo(2)];

    % Robot goes to the plane
    Mov_Cart_Inc(robCOMM,P(1),P(2),0,0,0,1,1)

    % Update robot position
    object_robo = [Plane(1), Plane(2), object_robo(3)];

    % Robot goes down
    Mov_Cart_Inc(robCOMM,0,0,-150,0,0,1,1)

    % Update robot position
    object_robo = [object_robo(1), object_robo(2), object_robo(3)-150];

    % Turn off vacuum
    Gripper_Open(robCOMM, 2);
end

%%% End program
fprintf("DONE\n");