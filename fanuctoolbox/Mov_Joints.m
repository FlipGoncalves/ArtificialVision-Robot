function Mov_Joints(robCOMM,J1,J2,J3,J4,J5,J6,Ty,Sp,Md)
% INPUTS: The TCP/IP comunication. The joint angles and the operation mode
% ( zero for syncronous mode and one for assyncronous mode)
% 
% DESCRIPTION: Move the robot throught joints.
% Ex: fprintf(robCOMM.handle, 'MOVTOJPOS\n-1 -56 37 -1 -113 12 0 1000 1')
%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------

string = num2str([J1 J2 J3 J4 J5 J6 Ty Sp Md]);           % Convert input parameters to string 
msg = sprintf('MOVTOJPOS\n%s', string);             % Send MOVTOJPOS command to robCOMM
fprintf(robCOMM.handle, msg);                       % return = OK or NOK

end
