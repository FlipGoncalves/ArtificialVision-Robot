function Comm_Close(robCOMM)
% INPUTS: The comunication that we want to close.
% 
% DESCRIPTION: This function will close the connection with the
% comunication
%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------


%CLOSE THE CONNECTION TO THE SERVER:
fclose(robCOMM.handle);             % Closes handle 
echotcpip('off');                   % Stop echo on port;
delete(robCOMM.handle);             % Deletes handle and robCOMM 
clear robCOMM; 

end
