%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------

%% TEST CONNECTIONS:

clear all; close all; clc;
%% OPEN THE PORT CONNECTION WITH MANIPULATOR HANDLER - FANUC (WORKING)

robCOMM = Comm_Open(4900,'192.168.0.231');      %IP of the FANUC [LR MATE 200ID - SMALL ONE]
%robCOMM = Comm_Open(4900,'192.168.0.230');       %IP of the FANUC [M-6 IBB 6S - BIGGER ONE]
%Note: To check if the connection was sucessfull robCOMM.handle      

%% CLOSE THE CONNECTION WITH FANUC FANUC
Comm_Close(robCOMM);

%% MOVING THE ROBOT BY CARTESIAN COORDINATES (ABSOLUTE)

%Mov_Cart_Abs(robCOMM,106,-2,273,-177,-15,-17,0,1,1,0,0,0,1,1000,1);            %Rest position
Mov_Cart_Abs(robCOMM,387,-3,245,179,-1,-16,0,1,1,0,0,0,1,1000,1);             %Test position
%Parameters: robCOMM, [X Y Z W P R R1 R2 R3 R4 R5 R6 Ty Sp Md]
%Note: To put this thing working you have to move first the damm joints!!!!

%% MOVING THE ROBOT BY JOINTS

Mov_Joints(robCOMM,0,-51,28,1,-103,344,0,500,1);                       %Rest position
%Mov_Joints(robCOMM,3,7,-2,1,-85,3,0,500,1);                             %Test position
%Parameters: robCOMM,[J1 J2 J3 J4 J5 J6 Md]

%% GETTING THE COORDINATES OF THE GRIPPER OF THE ROBOT: 

Pos = Get_Cart_Abs(robCOMM);                                            %Get the position of the hand in cartesian coordinates
%Joints = Get_Joints(robCOMM);                                           %Get the position of the hand in joints coordinates

%% DEFINES THE TRAJECTORY OR PATH OF THE ROBOT THEN IT MAKES HIM MOVE

P1=[106,-2,273,-177,-15,-17];
P2=[387,-3,245,179,-1,-16];
P3=P1;
P4=P2;
%Ex:  P1=[X Y Z W P R]

%Set_Path(robCOMM,P1,P2,P3,P4);
Mov_Path(robCOMM, 1000);                                                  %The second parameter is the speed

%% CALCULATE THE DIRECT/INVERSE KINEMATICS OF A GENERIC POSITION

%Pos = Get_DKin(robCOMM,0,-51,28,1,103,344);                    %It converst the joints coordiantes to cartesian coordinates.
%Parameters: robCOMM, [J1 J2 J3 J4 J5 J6]   

Joints = Get_IKin(robCOMM,388,-4,198,177,-1,-16);                         %It converst the cartesian coordinates to joints coordinates.
%Parameters: robCOMM, [X Y Z W P R]


%% OPEN/CLOSE HAND

%Gripper_Open(robCOMM);
%Gripper_Close(robCOMM);

%% CHECK IF THE POSITION IN CARTESIAN COORDIANTES IS IN RANGE OF THE ROBOT

%Status = Check_Pos(robCOMM,500,20,230,180,-4,6);                        % return = PosOk or PosNOK
Status = Check_Pos(robCOMM,10000000,20,230,180,-4,6); 
%Parameters: robCOMM, [X Y Z W P R]

%% MOVING THE ROBOT BY INCREMENTAL COORDINATES
Mov_Cart_Inc(robCOMM,0,10,30,1,-103,344,0)



