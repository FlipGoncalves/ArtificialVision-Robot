function Mov_Cart_Abs(robCOMM,X,Y,Z,W,P,R,R1,R2,R3,R4,R5,R6,Ty,Sp,Md)
% INPUTS: The port and the cartesian coordinates with parameters
% 
% DESCRIPTION: This robot will move in cartesian coordiantes (absolute)
% Ex: fprintf(robCOMM.handle, 'MOVTOCPOS\n53 0 294 177 -14 12 0 1 1 0 0 0 1 50 1')
%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------

string = num2str([X Y Z W P R R1 R2 R3 R4 R5 R6 Ty Sp Md]);     % Convert input parameters to string 
msg = sprintf('MOVTOCPOS\n%s', string);                         % Send MOVTOCPOS command to robCOMM
fprintf(robCOMM.handle, msg);                                   % return = OK or NOK

end
