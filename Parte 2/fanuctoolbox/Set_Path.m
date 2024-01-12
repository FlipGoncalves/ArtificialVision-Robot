function Set_Path(robCOMM,P1,P2,P3,P4)
% INPUTS: The TCP/IP comunication. The points P1=[[X Y Z W P R]]...
% 
% DESCRIPTION: Define the points were the robot will have to travel. After
% this function we have to use Mov_Path(robCOMM,Speed) to make him move
% true the path.
%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------

P=[P1 P2 P3 P4];                        %Creating the path with the points given
string = num2str(P);                    % Convert input parameters to string  
msg = sprintf('SETPATH\n%s', string);   % Send SETPATH command to robCOMM  
fprintf(robCOMM.handle, msg);           % return = [n] nodes defined


end
