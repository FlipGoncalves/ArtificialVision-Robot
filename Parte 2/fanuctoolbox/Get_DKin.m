function Pos = Get_DKin(robCOMM,J1,J2,J3,J4,J5,J6)
% INPUTS: The TCP/IP comunication. The joint angles
% 
% OUTPUT: Position of the robot in cartesian coordiantes in function of his
% joints
%
% DESCRIPTION: This function convert the joint angles to cartesian
% coordiantes. (Calculate the direct cinematic)
%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------

string = num2str([J1 J2 J3 J4 J5 J6]);              % Convert input parameters to string 
msg = sprintf('GETDIRKIN\n%s', string);             % Send GETDIRKIN command to robCOMM 
fprintf(robCOMM.handle, msg); 
bytesread = 1; n=0; bytestransmited=0; Pos = [];  msg=[];                % Returns the actual position
while (bytesread ~= 0)
    [str bytesread] = fgets(robCOMM.handle);
    if (bytesread == 31)
        n=n+1;
        Pos(n,:) = str2num(str(1:30));              % Returns position
    elseif (bytesread == 15)
        RED = str(1:14);                            % Returns configuration
    end
	bytestransmited=bytestransmited+1;              %Count if the number of bytes was OK
    msg=[msg str];                                  %Show the string with the messages
end

%msg
end
