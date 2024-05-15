% This file will permute all the robot arm states

% start by loading in the robot scheme
open_m = loadrobot("robotisOpenManipulator");

% File I will write to
fileID = fopen('test.csv', 'w');

% weights is the range of motion to consider
weights = [0 0 0 1 1 1];

% Make our inverse kintematics solver
ik = inverseKinematics("RigidBodyTree", open_m);

% Add header to file
fprintf(fileID, "%s,%s,%s,%s,%s,%s,%s\n", "x_pos", "y_pos", "z_pos", "joint1_angle", "joint2_angle", "joint3_angle", "joint4_angle");

x_pos = 0.3;
% iterate over all positions we care about
for y_pos = -0.2:0.01:0.2
    disp(y_pos);
    for z_pos = 0.0:0.01:0.4
        % Define our target position
        pos = [(x_pos + abs(y_pos)^1.5 + abs(z_pos-.2)^1.5) y_pos z_pos];
        poseTF = trvec2tform(pos);
        
        % TODO: Might want to consider changing guess to last position
        initialGuess = homeConfiguration(open_m);
        
        [configSoln,solnInfo] = ik("end_effector_link",poseTF,weights,initialGuess);
        fprintf(fileID, "%f,%f,%f,%f,%f,%f,%f\n", pos(1), pos(2), pos(3), configSoln(1).JointPosition, configSoln(2).JointPosition, configSoln(3).JointPosition, configSoln(4).JointPosition);
    end
end

fclose(fileID);