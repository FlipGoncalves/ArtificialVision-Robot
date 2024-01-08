function Mov_Cart_Inc(robCOMM,X,Y,Z,W,P,R,Md)
% INPUTS: The port and the cartesian coordinates.
% 
% DESCRIPTION: This robot will move in cartesian coordiantes (by increments)
%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------


string = num2str([X Y Z W P R Md]);              % Convert input parameters to string              
msg = sprintf('MOVTOCINC\n%s', string);          % Send MOVTOCINC command to robCOMM
fprintf(robCOMM.handle, msg);                    % return = OK or NOK

end
