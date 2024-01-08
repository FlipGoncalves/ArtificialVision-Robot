function WaitForFanucToStop(robCOMM)
disp('Waiting for robCOMM...')
warning off instrument:fgetl:unsuccessfulRead
bytesread=0; pause(0.1)
while bytesread==0
    [msg, bytesread]=fgetl(robCOMM.handle);
    pause(.1);
end
disp(['robCOMM repply: ' msg])