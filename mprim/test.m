% Using Ground coordinate system.
% Taking off and landing are considered totally different problem.
% using robot local coordinate system.

% some test scripts
% action time: 0.5 secs
% const acceleration for each action
% drift not implemented at present
% acceleration returned to 0 after each action.

% assume the value of acceleration to be constant, 3 levels: 0 1 2.
clear;
clc;

% generate the acceleration model;
% abs(a) * unit_direction vector;
% angle between unit_direction vector and x-o-y plane: phi
% angle between the projection on x-o-y plane and x-aixs: theta
% directions are discretized in 45 degrees
numofactions = 8 * 3 + 2;
acc = zeros(numofactions ,4);
a = 1;
for theta= 0:(pi/4):(pi*7/4)
    for phi = -(pi/4):(pi/4):(pi/4)
        acc(a,1) = cos(phi)*cos(theta);     % x
        acc(a,2) = cos(phi)*sin(theta);     % y
        acc(a,3) = sin(phi);                % z
                                            % yaw acceleration not implemented.
        a = a+1;
    end
end

acc(a,3) = 1;          % acceleration in z-axis direction
acc(a+1,3) = -1;

accvalue=[4 8];
acc = acc*accvalue(2);

acc = round(acc, 4);
for i = 1:numofactions
    quiver3(0,0,0,acc(i,1), acc(i,2), acc(i,3));
    hold on;
end
grid on;
xlabel('x');
ylabel('y');
zlabel('z');


prompt = 'Press Enter to continue: ';
m = input(prompt);
close;

% ****************************************************
% ******************* stage one **********************
% pstart = [px, py, pz, pyaw];
% vstart = [vx, vy, vz, pyaw];
% one unit per/sec as start velocity

pstart = [0,0,0,0];
vstart = [2,0,0,0];
pend = zeros(1,4);
vend = zeros(1,4);


% run exhaustively until no new situation occurrs
% sum of the absolute values should not exceed 3;

for i = 1:numofactions
    effective = 0;
    [temppend, tempvend, effective] = action_plot(pstart, vstart, acc(i,:));
    if 1 == effective
        pend = [pend; temppend];
        vend = [vend; tempvend];
    end
end
pend = pend(2:end,:);
vend = vend(2:end,:);
pend = round(pend, 4);
vend = round(vend, 4);

%grid on;
%box on;
%ax =gca;
%ax.BoxStyle = 'full';
%xlabel('x');
%ylabel('y');
%zlabel('z');
%prompt = 'Press Enter to continue: ';
%m = input(prompt);
%close;


% ****************************************************
% ******************* stage two **********************
% based on the speed generated before
pend1 = zeros(1,4);
vend1 = zeros(1,4);
[row,column] = size(vend);

run_forever = 1;
while( 0 ~= run_forever)
   
    for j = 1:row
        vstart = vend(j,:);
        pstart = pend(j,:);
        temppend1 = zeros(1,4);
        tempvend1 = zeros(1,4);
        for i = 1:numofactions
            effective = 0;
            [ temppend, tempvend, effective] = action_plot(pstart, vstart, acc(i,:));
            if 1 == effective
                temppend1 = [temppend1; temppend];
                tempvend1 = [tempvend1; tempvend];
            end
        end
        temppend1 = temppend1(2:end, :);
        tempvend1 = tempvend1(2:end, :);
        temppend1 = round(temppend1, 4);
        tempvend1 = round(tempvend1, 4);

        %grid on;
        %box on;
        %ax =gca;
        %ax.BoxStyle = 'full';
        %xlabel('x');
        %ylabel('y');
        %zlabel('z');
        %prompt = 'Press Enter to continue: ';
        %m = input(prompt);
        %close;

        pend1 = [pend1; temppend1];
        vend1 = [vend1; tempvend1];

    end
    
    vend1 = round(vend1, 4);
    pend1 = round(pend1, 4);
    vend1 = unique(vend1,'rows');
    pend1 = unique(pend1,'rows');
    if( size(vend1) == size(vend) )     % no new element introduced to it
        break;
    else
        vend = vend1;
        pend = zeros(size(pend1));
        pend(:,4) = pend1(:,4);
    end

    run_forever = run_forever + 1;
    if run_forever > 100
        break;
    end

end
