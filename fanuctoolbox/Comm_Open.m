function robCOMM = Comm_Open(port, ip)
% INPUTS: The port and IP of the comunication.
% 
% OUTPUT: This functions gives us a robCOOM wich is a variable with the
% comunication.
%
% DESCRIPTION: This function will create a comunication TCP/IP
%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------

robCOMM.port = port;                                % Defines the COMM variables
robCOMM.ip = ip;       
robCOMM.handle=tcpclient(robCOMM.ip, robCOMM.port);     % Creates the tcpip object
set(robCOMM.handle, 'Timeout', 1);                  % 1 sec timeout
fopen(robCOMM.handle);                              % Opens the object
end


