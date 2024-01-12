function Gripper_Open(robCOMM,robot)
% INPUTS: The TCP/IP comunication. 
%         The robot that you want to use 1 for the bigger one and two for the smaller one.         
%
% DESCRIPTION: This function will open the Hand of the robot.
%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------

if robot==1
    tp = 'GRIPPERO';                        % Opens the Fanuc gripper  
    msg = sprintf('RUNTPP\n%s', tp);
    fprintf(robCOMM.handle, msg);           % return = TP OK or TP NOK
end

if robot==2
    fprintf(robCOMM.handle, 'SETDIO\n2 7 1');
end
end
