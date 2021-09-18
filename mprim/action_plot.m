% plot function
% input is acceleration, start vlocity, end velocity
% pstart, vstart, acc, arc 1x4 matrices
% if one action is effective, effective value = 1
function[pend, vend, effective] = action_plot(pstart, vstart, accOri)

% time configuration
T = 0.5;
TS = 0.025;
size = 499;
deltaT = T/size;
t = 0:deltaT:T;
ts = t.^2;
maxspeed = 14.1;      % 3 times of acceleration in one direction
minspeed = 2;

% % calculation of start and end;
% rotation of acceleration based on pstart direction
% rotate back to the ground coordinate system
angle = -pstart(4);

Rz = rotate_z(angle);
% the ground acceleration direction
acc = accOri*Rz;


flag = 0;   
% discretize action over time
% the acc vector should be roated according to current orientation.
trajectory = zeros(500, 4);
speed = zeros(500, 4);
speed( 1, :) = vstart;
trajectory(1, :) = pstart;

for ind = 2:(size+1)

    speed(ind, :) = speed(ind-1, :) + acc *deltaT; 
    check = sqrt( sum(speed(ind,:).^2));
    if (check < minspeed || check > maxspeed)       % assure the speed within range
        flag = 1;
        break;
    else
        
        trajectory(ind, :) = trajectory(ind-1, :) + speed(ind-1, :) *deltaT + 0.5 * acc * deltaT.^2;
        trajectory(ind, 4) = acos( speed(ind,1) / sqrt( speed(ind,1)^2 + speed(ind,2)^2 ) );

        if( trajectory(ind, 2) <  0)
            trajectory(ind,4) = 2*pi - trajectory(ind,4);
        end
       
        angle = -trajectory(ind, 4);
        Rz = rotate_z(angle);
        acc = accOri*Rz;
    end

end


if 0 == flag 
    effective = 1;
    vend = speed(size +1,:);
    pend = trajectory(size+1, :);
    %plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'r');
    %hold on;
else 
    effective = 0;
    vend = vstart;
    pend = pstart;
end

