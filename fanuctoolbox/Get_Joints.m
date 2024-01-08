function Joints = Get_Joints(robCOMM)
% INPUTS: The TCP/IP comunication.
% 
% OUTPUT: Position of the robot by his joints
%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------


fprintf(robCOMM.handle, 'GETCRJPOS');       % Send GETCRJPOS command to robCOMM 
bytesread = 1; n=0; bytestransmited=0; Joints = [];  msg=[];                % Returns the actual position
while (bytesread ~= 0)
    [str bytesread] = fgets(robCOMM.handle);
    if (bytesread == 51)
        n=n+1;
        Joints(n,:) = str2num(str(1:50));              % Returns position
    elseif (bytesread == 15)
        RED = str(1:14);                            % Returns configuration
    end
	bytestransmited=bytestransmited+1;              %Count if the number of bytes was OK
    msg=[msg str];                                  %Show the string with the messages
end


%msg
%Joints = [msg];               % Returns the actual joint configuration

end
