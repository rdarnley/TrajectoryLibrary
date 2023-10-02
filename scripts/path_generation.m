clc;
clear all;
close all;

dis = 3.0;
angle = 27;
deltaAngle = angle / 3;
scale = 0.65;

pathStartAll = zeros(4, 0);
pathAll = zeros(5, 0);
pathList = zeros(5, 0);
pathID = 0;
groupID = 0;

figure;
hold on;
box on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');

fprintf('\nGenerating paths\n');

##for shift1 = -angle : deltaAngle : angle

    shift1 = 0;
    wayptsStart = [0, 0, 0;
                   dis, shift1, 0];

    pathStartR = 0 : 0.1 : 1;
    pathStartShift = spline(wayptsStart(:, 1), wayptsStart(:, 2), pathStartR);

    pathStartX = pathStartR .* cos(pathStartShift * pi / 180);
    pathStartY = pathStartR .* sin(pathStartShift * pi / 180);
    pathStartZ = zeros(size(pathStartX));

    pathStart = [pathStartX; pathStartY; pathStartZ; ones(size(pathStartX)) * groupID];
    pathStartAll = [pathStartAll, pathStart];

    for shift2 = -angle * scale + shift1 : deltaAngle * scale : angle * scale + shift1
        for shift3 = -angle * scale^2 + shift2 : deltaAngle * scale^2 : angle * scale^2 + shift2

##                Short range traj
                waypts = [pathStartR', pathStartShift', pathStartZ';
                          2, shift2, 0;
                          3 - 0.001, shift3, 0;
                          3, shift3, 0];

                pathR = 0 : 0.1 : waypts(end, 1);
                pathShift = spline(waypts(:, 1), waypts(:, 2), pathR);

                pathX = pathR .* cos(pathShift * pi / 180);
                pathY = pathR .* sin(pathShift * pi / 180);
                pathZ = zeros(size(pathX));

                path = [pathX; pathY; pathZ; ones(size(pathX)) * pathID; ones(size(pathX)) * groupID];
                pathAll = [pathAll, path];
                pathList = [pathList, [pathX(end); pathY(end); pathZ(end); pathID; groupID]];

                pathID = pathID + 1;

                plot3(pathX, pathY, pathZ);

##     Medium range trajectories
                waypts = [pathStartR', pathStartShift', pathStartZ';
                          4, shift2, 0;
                          6 - 0.001, shift3, 0;
                          6, shift3, 0];

                pathR = 0 : 0.1 : waypts(end, 1);
                pathShift = spline(waypts(:, 1), waypts(:, 2), pathR);

                pathX = pathR .* cos(pathShift * pi / 180);
                pathY = pathR .* sin(pathShift * pi / 180);
                pathZ = zeros(size(pathX));

                path = [pathX; pathY; pathZ; ones(size(pathX)) * pathID; ones(size(pathX)) * groupID];
                pathAll = [pathAll, path];
                pathList = [pathList, [pathX(end); pathY(end); pathZ(end); pathID; groupID]];

                pathID = pathID + 1;

                plot3(pathX, pathY, pathZ);


## Long range trajectories

                waypts = [pathStartR', pathStartShift', pathStartZ';
                          6, shift2, 0;
                          8 - 0.001, shift3, 0;
                          8, shift3, 0];

                pathR = 0 : 0.1 : waypts(end, 1);
                pathShift = spline(waypts(:, 1), waypts(:, 2), pathR);

                pathX = pathR .* cos(pathShift * pi / 180);
                pathY = pathR .* sin(pathShift * pi / 180);
                pathZ = zeros(size(pathX));

                path = [pathX; pathY; pathZ; ones(size(pathX)) * pathID; ones(size(pathX)) * groupID];
                pathAll = [pathAll, path];
                pathList = [pathList, [pathX(end); pathY(end); pathZ(end); pathID; groupID]];

                pathID = pathID + 1;

                plot3(pathX, pathY, pathZ);
        end
    end

    groupID = groupID + 1


pathID

fileID = fopen('paths.ply', 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathAll, 2));
fprintf(fileID, 'property float x\n');
fprintf(fileID, 'property float y\n');
fprintf(fileID, 'property float z\n');
fprintf(fileID, 'property int path_id\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d %d\n', pathAll);
fclose(fileID);




fprintf('\nProcessing complete\n');
%}
