function cameraParams = calibrateCamera(win, wbcm)
    
    if win == 0
        videoOS = 'linuxvideo';
    else
        videoOS = 'winvideo';
    end

    if exist('camera_params.mat', 'file') == 2
        cameraParams = load('camera_params.mat').cameraParams;
    else

        info = imaqhwinfo(videoOS);
        camnum = 1;         % numero da camara do bus (USB,firewire)
        image_type = 'YUY2_1280x720'; %Depende da camara, usar o melhor depois a testar
        
        cam = webcam(wbcm);
        CC = [];
        NFC = 15;
        
        try 
            for i=1:NFC
                img = snapshot(cam);
                imwrite(img,'image_calibration_' + string(i) + '.png');
                pause(0.5)
            end
        catch 
            Err = 'Oops'
        end
        
        imageFileNames = { 'image_calibration_1.png','image_calibration_2.png','image_calibration_3.png','image_calibration_4.png','image_calibration_5.png',...
                           'image_calibration_6.png','image_calibration_7.png','image_calibration_8.png','image_calibration_9.png','image_calibration_10.png',...
                           'image_calibration_11.png','image_calibration_12.png','image_calibration_13.png','image_calibration_14.png','image_calibration_15.png',};
        delete(cam)
        
        detector = vision.calibration.monocular.CheckerboardDetector();
        [ imagePoints , imagesUsed ] = detectPatternPoints( detector , imageFileNames );
        imageFileNames = imageFileNames( imagesUsed );
        % Read the first image to obtain image size
        originalImage = imread( imageFileNames {1});
        [ mrows , ncols , ~] = size ( originalImage );
        % Generate world coordinates for the planar pattern keypoints
        squareSize = 25; % in units of ' millimeters '
        worldPoints = generateWorldPoints( detector, 'SquareSize', squareSize );
    
        % Calibrate the camera
        [cameraParams,imagesUsed,estimationErrors]=estimateCameraParameters(...
        imagePoints,worldPoints,'EstimateSkew',false,'EstimateTangentialDistortion',false,...
        'NumRadialDistortionCoefficients',2,'WorldUnits','millimeters',...
        'InitialIntrinsicMatrix',[],'InitialRadialDistortion',[],'ImageSize',[mrows,ncols]);
    
        % save camera parameters
        save('camera_params.mat', 'cameraParams');


        try
            for i=1:NFC
                delete('image_calibration_' + string(i) + '.png')
            end
        catch
        end
    end

    % Visualize pattern locations
    % h2 = figure ; showExtrinsics(cameraParams, 'CameraCentric' );
end

