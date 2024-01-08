function [Preal]=Converter_Inc_Abs(Xs,Ys,Zs,Xr,Yr,Zr)
% CONVERT INCREMENTAL POSITION TO ABSOLUTE COORDINATES
% This function do what the function incremental can´t do by itself.
% Because the handler always says out of range without any special reason.
% 
% INPUTS: The coordinates in incremtal position and Rest position(the rest
% position simbolize the position (0,0) were want the robot start moving
% from.
% OUTPUT: Gives the coordinates back in absolute to thw robot be able
% tomove withotu giving any kind of errors.
%
% Version 1, Tomiła Tyczyńska, Daniel Costa, 2015, LAR, UA, Portugal.
%--------------------------------------------------------------------

Xfinal=Xs+Xr;
Yfinal=Ys+Yr;
Zfinal=Zs+Zr;

Preal=[Xfinal Yfinal Zfinal];


end

