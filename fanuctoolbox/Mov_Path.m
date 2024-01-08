function Mov_Path(robCOMM, Speed)
% INPUTS: The TCP/IP comunication. The speed of movement.
% 
% DESCRIPTION: Move the robot by the Path set in the function Set_Path.
%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------

string = num2str(Speed);                        % Convert input parameters to string
msg = sprintf('MOVTHPTH\n%s', string);          % Send MOVTHPTH command to robCOMM
fprintf(robCOMM.handle, msg);                   % return = Move OK or NOK

end
