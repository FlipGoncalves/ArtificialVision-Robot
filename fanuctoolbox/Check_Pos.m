function Status = Check_Pos(robCOMM,X,Y,Z,W,P,R)
% INPUTS: The communication. The X, Y, Z coordianates and de Euler angles.
% Everything must be on the same vector. Ex: [X Y Z W P R]
% 
% OUTPUT: Indicates if the points indicated are the space work of the
% robot.
%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------


string = num2str([X Y Z W P R]);            % Convert input parameters to string
msg = sprintf('CHECKCPOS\n%s', string);     % Send CHECKCPOS command to robCOMM
fprintf(robCOMM.handle, msg);               % return = PosOk or PosNOK
Status=fgets(robCOMM.handle);
end
