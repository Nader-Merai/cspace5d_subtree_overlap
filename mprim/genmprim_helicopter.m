%
%  This program is used to generate the highspeed motion primitive 
%  for an airplane(probably a fixed wing).
%  And saves them into a file.
%  The current version consider the x,y,z direction speed to be constant.
%  The motion consists of translational and rotational movement. 
%
%  No turn in place allowed for the airplane. 
%  No climb in place allowed for the airplane. 
%  No backward movement allowed for the airplane.
%  Drift is not considered at present.
%  The angular speed returned to 0 after every action.
%
%  Version: 1.0
%  By: Wei Du
%  Date: 05/19/2018
%

function[] = genmprim(outfilename)


% generate the motion primitive and visualize it 

resolution = 0.250;     % resolution 0.025
numberofangles = 32;    % preferably a power of 2, definitely multiple of 8
numofprimsperangle = 16;

% multipliers(multiplier is used as costmult*cost)




% Save the motion primitive into a file
