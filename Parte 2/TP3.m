%%% Add tools to path
addpath 'fanuctoolbox'

%%% Clear variables
clc
close all
clear

%%% Calibrate Camera
cameraParams = calibrateCamera(1);   % 1 for windows / 0 for linux 

%%% Connect to robot
% Define communication variables
robCOMM.port = 4900;
robCOMM.ip = '192.168.0.229';

% Create the TCP IP object
robCOMM.handle = tcpclient(robCOMM.ip, robCOMM.port);
% 1 sec timeout
set(robCOMM.handle, 'Timeout', 1);

% Open the object
fopen(robCOMM.handle);

%%% Find Objects and Move the Robot
cam = webcam(2);

image = snapshot(cam);
[im, newOrigin] = undistortImage(image, cameraParams, 'OutputView', 'full');
[imagePoints, boardSize] = detectCheckerboardPoints(im);
[R, t] = extrinsics(imagePoints, cameraParams.WorldPoints, cameraParams);

% figure;
% imshow(im); hold on; plot(imagePoints(:,1), imagePoints(:,2), 'g*');
% plot(imagePoints(1,1), imagePoints(1,2), 'sr');
% plot(imagePoints(1,1)+10, imagePoints(1,2), 'sy');
% plot(imagePoints(1,1), imagePoints(1,2)+10, 'sb');

[H,S,V] = rgb2hsv(im);

greenObjectsMask = (H > 0.2 & H < 0.5 & S > 0.2 & S < 0.8 & V > 0.2);
redObjectsMask   = (H > 0.7 & H < 1.0 & S > 0.5 & S < 1.0 & V > 0.2);
blueObjectMask   = (H > 0.5 & H < 0.8 & S > 0.5 & S < 1.0 & V > 0.2);
YellowObjectMask = (H > 0.0 & H < 0.2 & S > 0.5 & S < 1.0 & V > 0.2);

% Label connected components in the binary mask
labeledRedImage = bwlabel(redObjectsMask);
labeledGreenImage = bwlabel(greenObjectsMask);
labeledBlueImage = bwlabel(blueObjectsMask);
labeledYellowImage = bwlabel(yellowObjectsMask);

% Calculate region properties, including centroids
Redstats = regionprops(labeledRedImage, 'Centroid', 'Area');
Greenstats = regionprops(labeledGreenImage, 'Centroid', 'Area');
Bluestats = regionprops(labeledBlueImage, 'Centroid', 'Area');
Yellowstats = regionprops(labeledYellowImage, 'Centroid', 'Area');

% Find the index of the region with the largest area
[~, maxRedAreaIndex] = max([Redstats.Area]);
[~, maxGreenAreaIndex] = max([Greenstats.Area]);
[~, maxBlueAreaIndex] = max([Bluestats.Area]);
[~, maxYellowAreaIndex] = max([Yellowstats.Area]);

% Get the centroid of the region with the largest area
largestRedAreaCentroid = Redstats(maxRedAreaIndex).Centroid;
largestGreenAreaCentroid = Greenstats(maxGreenAreaIndex).Centroid;
largestBlueAreaCentroid = Greenstats(maxBlueAreaIndex).Centroid;
largestYellowAreaCentroid = Yellowstats(maxYellowAreaIndex).Centroid;

Points = [largestRedAreaCentroid; largestGreenAreaCentroid; largestBlueAreaCentroid; largestYellowAreaCentroid  ];

for i=1:size(Points, 1)
    
    input("Press <ENTER> to continue")

    % Get robot position
    fprintf(robCOMM.handle, 'GETCRCPOS');

    % Return the actual position
    bytesread = 1;
    n=0;
    k=0;
    Pos = [];
    while (bytesread ~= 0)
        [str bytesread] = fgets(robCOMM.handle);
        if (bytesread == 31)
            n=n+1;
            % Returns position
            Pos(n,:) = str2num(str(1:30));
        elseif (bytesread == 15)
            % Returns configuration
            RED = str(1:14);
        end
	    k=k+1;
    end
    Pos

    T = [0 1 0  335
         1 0 0 -445
         0 0 0 -160
         0 0 0  0];

    object = [pointsToWorld(cameraParams, R, t, [Points(i,1), Points(i,2)]) 1 1];
    object_robo = T * object'
    P = [object_robo(1)-Pos(1,1), object_robo(2)-Pos(1,2)]

    % Turn On Vacuum
    Gripper_Close(robCOMM, 2);
    pause(1);
    % input("Press <ENTER> to continue")

    % Robot goes down to pick up object
    Pz = 164-Pos(1,3);
    Mov_Cart_Inc(robCOMM,P(1),P(2),164-Pos(1,3),0,0,1,1)
    pause(5);
    % input("Press <ENTER> to continue")

    % Robot goes Up
    Mov_Cart_Inc(robCOMM,0,0,-160-Pz,0,0,1,1)
    pause(5);
    % input("Press <ENTER> to continue")

    % Turn Off Vacuum
    Gripper_Open(robCOMM, 2);
    pause(1);
    % input("Press <ENTER> to continue")
end

fprintf("DONE\n");

