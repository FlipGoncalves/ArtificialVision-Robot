function Joints = Get_IKin(robCOMM,X,Y,Z,W,P,R)
% INPUTS: The TCP/IP comunication. The cartesian coordinates
% 
% OUTPUT: Position of the robot in joints in function of cartesian
% coordinates
%
% DESCRIPTION: This function convert the cartesian coordiantes to joint
% angles. (calculate the inverse kinematics)
%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------

 string = num2str([X Y Z W P R]);           % Convert input parameters to string
 msg = sprintf('GETREVKIN\n%s', string);    % Send GETREVKIN command to robCOMM
 fprintf(robCOMM.handle, msg);
bytesread = 1; n=0; bytestransmited=0; Joints = [];  msg=[];                % Returns the actual position
while (bytesread ~= 0)
    [str bytesread] = fgets(robCOMM.handle);
    if (bytesread == 31)
        n=n+1;
        Joints(n,:) = str2num(str(1:30));              % Returns position
    elseif (bytesread == 15)
        RED = str(1:14);                            % Returns configuration
    end
	bytestransmited=bytestransmited+1;              %Count if the number of bytes was OK
    msg=[msg str];                                  %Show the string with the messages
end
 msg

end
