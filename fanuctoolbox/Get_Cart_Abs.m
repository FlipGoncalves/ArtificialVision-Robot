function Pos = Get_Cart_Abs(robCOMM)
% INPUTS: The TCP/IP comunication.
% 
% OUTPUT: Position of the robot in cartesian coordianates
%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------


fprintf(robCOMM.handle, 'GETCRCPOS');   % Send GETCRCPOS command to robCOMM
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
msg
% %EXAMPLE:
% msg =
% 
% Moving...
% 1
% 1
% Current position
%     93.598    -1.360   318.588
%   -176.650   -14.659   -16.784
% N U T, 0, 0, 1
end
