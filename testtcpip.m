% clear variables
clc
close all
clear

% robCOMM server needs this time to go back listening
pause(4);
        
% Define communication variables
port=4900;
ip='192.168.0.229';

% Define robCOMM variables
robCOMM.port = port;                                    
robCOMM.ip = ip;      

% Creates TCP IP object
robCOMM.handle = tcpclient(robCOMM.ip, robCOMM.port);
set(robCOMM.handle,'Timeout',1); %1 sec timeout

% Open the object and get robot position
fopen(robCOMM.handle);
fprintf(robCOMM.handle, 'GETCRCPOS\n');

%read text Current position
[msg, bytesread]=fgetl(robCOMM.handle);
%read xyz and convert to numbers
[msg, bytesread]=fgetl(robCOMM.handle);
[XYZ]=str2num(msg);
%read wpr and convert to numbers
[msg, bytesread]=fgetl(robCOMM.handle);
[WPR]=str2num(msg);
%read redundancy and convert to numbers
[msg, bytesread]=fgetl(robCOMM.handle);
if (msg(1)=='F') RED(1)=1; else RED(1)=0; end
if (msg(3)=='U') RED(2)=1; else RED(2)=0; end
if (msg(5)=='T') RED(3)=1; else RED(3)=0; end
[RED(4:6)]=str2num(msg(6:size(msg,2)));

% Move the Robot
Ty = 0; Sp = 500; Md = 0;
Mov_Cart_Abs( ...
        robCOMM,...                                     % handler
        XYZ(1),XYZ(2),XYZ(3),...                        % xyz cartesian
        WPR(1),WPR(2),WPR(3),...                        % xyz rotation
        RED(1),RED(2),RED(3),RED(4),RED(5),RED(6),...   % redundancies
        Ty,...                                          % motion Type (0 - Linear | 1 - Joint)
        Sp,...                                          % motion Speed (0 - mm/sec | 1 - %)
        Md...                                           % operation Mode (0 - sync | 1 - async)
    )

% Wait for Robot to Stop
WaitForFanucToStop(robCOMM)
WaitForFanucToStop(robCOMM)

% Stop TCP IP communication
fclose(robCOMM.handle);
delete(robCOMM.handle);
clear robCOMM;







