clc;clear all; close all; dbstop if error;
addpath(' /home/binbin/disk/project-code/lane_quality_assessment/ICNet-master/LaneRecognitionTracking/matlab/bspline');
%%%%%%%%%%%%%%%%%%%%%%%%%

%%0056
% vioPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0056/Lanes/vio.txt';
% worldCenterPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0056/Lanes/worldCenterCoordinates.txt';
% worldLeftPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0056/Lanes/worldLeftCoordinates.txt';
% worldRightPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0056/Lanes/worldRightCoordinates.txt';

%%0035
% vioPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0035/0035_Lanes/2011_09_26_0035-vio.txt';
% worldCenterPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0035/0035_Lanes/worldCenterCoordinates.txt';
% worldLeftPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0035/0035_Lanes/worldLeftCoordinates.txt';
% worldRightPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0035/0035_Lanes/worldRightCoordinates.txt';

%%0051
% vioPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0051/0051_Lanes/vio.txt';
% worldCenterPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0051/0051_Lanes/worldCenterCoordinates.txt';
% worldLeftPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0051/0051_Lanes/worldLeftCoordinates.txt';
% worldRightPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0051/0051_Lanes/worldRightCoordinates.txt';

%%0039
% vioPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0039/vio.txt';
% worldCenterPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0039/Lanes/worldCenterCoordinates.txt';
% worldLeftPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0039/Lanes/worldLeftCoordinates.txt';
% worldRightPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0039/Lanes/worldRightCoordinates.txt';


%%0023
% vioPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0023/vio.txt';
% worldCenterPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0023/Lanes/worldCenterCoordinates.txt';
% worldLeftPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0023/Lanes/worldLeftCoordinates.txt';
% worldRightPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0023/Lanes/worldRightCoordinates.txt';

%0104
vioPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0104/vio.txt';
worldCenterPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0104/Lanes/worldCenterCoordinates.txt';
worldLeftPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0104/Lanes/worldLeftCoordinates.txt';
worldRightPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0104/Lanes/worldRightCoordinates.txt';


% compute length
slamat = readtable(vioPath);
RotTransMat = table2cell(slamat);
RotTransMat = cell2mat(RotTransMat(:, 1:12));
tranjectory = 0;
trans = [RotTransMat(:, 12), RotTransMat(:, 4), RotTransMat(:, 8)];
for i = 2 : 1 : size(trans, 1)
    tranjectory = tranjectory + norm(trans(i, :) - trans(i -1, :));
end
display(tranjectory);

zvalue = [trans(:, 3)];
%zvalue = sort(zvalue,'descend');

%display lbs and lcc
LccCtrlPts = readtable(worldCenterPath);
LccCtrlPtsMat = table2array(LccCtrlPts);
LccCtrlPtsMat = postProcess(LccCtrlPtsMat, trans);
LccCtrlPtsMat(:, 2) = - LccCtrlPtsMat(:, 2);
LccCtrlPtsMat(:, 3) = zvalue( randi([1, size(LccCtrlPtsMat, 1)], size(LccCtrlPtsMat, 1), 1));

LeftCtrlPts = readtable(worldLeftPath);
LeftCtrlPtsMat = table2array(LeftCtrlPts);
LeftCtrlPtsMat = postProcess(LeftCtrlPtsMat, trans);
LeftCtrlPtsMat(:, 2) = - LeftCtrlPtsMat(:, 2);
LeftCtrlPtsMat(:, 3) = zvalue( randi([1, size(LeftCtrlPtsMat, 1)], size(LeftCtrlPtsMat, 1), 1));

RightCtrlPts = readtable(worldRightPath);
RightCtrlPtsMat = table2array(RightCtrlPts);
RightCtrlPtsMat = postProcess(RightCtrlPtsMat, trans);
RightCtrlPtsMat(:, 2) = - RightCtrlPtsMat(:, 2);
RightCtrlPtsMat(:, 3) = zvalue( randi([1, size(RightCtrlPtsMat, 1)], size(RightCtrlPtsMat, 1), 1));

numfCtrlPts = min(size(LccCtrlPtsMat, 1), min(size(LeftCtrlPtsMat, 1), size(RightCtrlPtsMat, 1)));

% get the spline curve
order = 5;
ptselect = 1:10: numfCtrlPts;
LccCtrlPtSpline = computeSpline(order, ptselect, numfCtrlPts, LccCtrlPtsMat);
LeftCtrlPtsSpline = computeSpline(order, ptselect, numfCtrlPts, LeftCtrlPtsMat);
RightCtrlPtSpline = computeSpline(order, ptselect, numfCtrlPts, RightCtrlPtsMat);


figure; hold on; view(-45, 60); grid on; box on;
xlabel('x'); ylabel('y'); zlabel('z');
set(gca,'FontSize',24);
plot3(LccCtrlPtSpline(1,:), LccCtrlPtSpline(2,:), LccCtrlPtSpline(3,:), 'b');
plot3(LeftCtrlPtsSpline(1,:), LeftCtrlPtsSpline(2,:), LeftCtrlPtsSpline(3,:), 'g');
plot3(RightCtrlPtSpline(1,:), RightCtrlPtSpline(2,:),RightCtrlPtSpline(3,:),  'g');


function splinePts = computeSpline(order,ptselect,  numfCtrlPts, CtrlPtSpline)
knots = [0, 0, 0, 0, linspace(0, 1, size(ptselect, 2) -2),  1, 1, 1];
LccSplinexy = bspline_deboor(order, knots, CtrlPtSpline(ptselect, 1:2)');
LccSplinexz = bspline_deboor(order, knots, [CtrlPtSpline(ptselect, 1), CtrlPtSpline(ptselect, 3)]');
splinePts = [LccSplinexy; LccSplinexz(2, :)];
% temp = [splinePts(1, 1), splinePts(1, 2), splinePts(1, 3)];
% temp = [temp; splinePts'];
% splinePts = temp';
end

function newPts = postProcess(ctrlPts, trans)
newPts = ctrlPts;
newPts(:, 3) = ( ( ctrlPts(:, 3) - min(ctrlPts(:, 3)) )/ ( max(ctrlPts(:, 3)) - min(ctrlPts(:, 3)) )) * (max(trans(:, 3)) - min(trans(:, 3))) + min(trans(:, 3)) ;
newPts(:, 3) =min(newPts(:, 3))*ones(size(newPts(:, 3), 1), 1);
end