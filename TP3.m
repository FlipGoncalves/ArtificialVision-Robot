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

Points = [];
image = snapshot(cam);
[image, newOrigin] = undistortImage(image, cameraParams, 'OutputView', 'full');
[imagePoints, boardSize] = detectCheckerboardPoints(image);

[R, t] = extrinsics(imagePoints, cameraParams.WorldPoints, cameraParams);

% Extract the individual color channels
redChannel = image(:,:,1); % Red channel
greenChannel = image(:,:,2); % Green channel
blueChannel = image(:,:,3); % Blue channel

% Create a binary mask for red and greend objects based on a threshold
threshold = 100;
redObjectsMask = (redChannel > threshold) & (greenChannel < threshold) & (blueChannel < threshold);
greenObjectsMask = (greenChannel > threshold) & (redChannel < threshold) & (blueChannel < threshold);

% Label connected components in the binary mask
labeledRedImage = bwlabel(redObjectsMask);
labeledGreenImage = bwlabel(greenObjectsMask);

% Calculate region properties, including centroids
Redstats = regionprops(labeledRedImage, 'Centroid', 'Area');
Greenstats = regionprops(labeledGreenImage, 'Centroid', 'Area');

% Find the index of the region with the largest area
[~, maxRedAreaIndex] = max([Redstats.Area]);
[~, maxGreenAreaIndex] = max([Greenstats.Area]);

% Get the centroid of the region with the largest area
largestRedAreaCentroid = Redstats(maxRedAreaIndex).Centroid;
largestGreenAreaCentroid = Greenstats(maxGreenAreaIndex).Centroid;

subplot(1, 2, 1);
imshow(image);
title('Red Objects');
hold on;
plot(largestRedAreaCentroid(1), largestRedAreaCentroid(2), 'r*'); % Mark centroid with a red asterisk

subplot(1, 2, 2);
imshow(image);
title('Green Objects');
hold on;
plot(largestGreenAreaCentroid(1), largestGreenAreaCentroid(2), 'r*'); % Mark centroid with a red asterisk

Points = [largestRedAreaCentroid; largestGreenAreaCentroid]
pause()

for i=1:size(Points, 1)

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

    object = pointsToWorld(cameraParams, R, t, [Points(i,1), Points(i,2)])
    P = [Pos(1,1)-object(1), Pos(1,2)-object(2)]

    % Move the Robot
    % Mov_Cart_Inc(robCOMM,P(1),P(2),164-Pos(1,3),0,0,1,1)
    % Robot goes down to pick up object
    Mov_Cart_Inc(robCOMM,0,0,0-Pos(1,3),0,0,1,1)
    pause(1)
    % Gripper_Open(robCOMM, 1);
    pause(1)
    % Robot goes Up
    Mov_Cart_Inc(robCOMM,0,0,160,0,0,1,1)
    pause(1)
end

fprintf("DONE\n");

