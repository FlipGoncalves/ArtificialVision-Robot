function Gripper_Close(robCOMM,robot)
% INPUTS: The TCP/IP comunication. 
%
% DESCRIPTION: This function will close the Hand of the robot.
%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------

if robot==1
    tp = 'GRIPPERC';                % Closes the Fanuc gripper
    msg = sprintf('RUNTPP\n%s', tp);
    fprintf(robCOMM.handle, msg);       % return = TP OK or TP NOK
end
 
if robot==2
      fprintf(robCOMM.handle, 'SETDIO\n2 7 0');
end

end
