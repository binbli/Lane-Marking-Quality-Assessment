%
plot surface
clear all;
addpath('C:\Users\Ankit\Documents\TAMU Succeed\Research\Autonomous Driving\devkit\matlab');
model = 'Spline';
imgDir = 'C:\Users\Ankit\Documents\TAMU Succeed\Research\Autonomous Driving\Lidar\';
outputDir = 'C:\Users\Ankit\Documents\TAMU Succeed\Research\Autonomous Driving\Lanes\';
modelsFile = [model 'Models.txt'];
calib_dir = 'C:\Users\Ankit\Documents\TAMU Succeed\Research\Autonomous Driving\calib';
cam = 2;
imgPrefix = 'intersected_';
outputPrefix = [model '_'];
stride = 1;
fM = fopen(modelsFile);

lineM = fgetl(fM);
calib = loadCalibrationCamToCam(fullfile(calib_dir, 'calib_cam_to_cam.txt'));
Tr_velo_to_cam = loadCalibrationRigid(fullfile(calib_dir, 'calib_velo_to_cam.txt'));

%
compute projection
matrix velodyne
->
image plane
R_cam_to_rect = eye(4);
R_cam_to_rect(1:3, 1:3) = calib.R_rect{
1
};

P_velo_to_img = calib.P_rect { cam + 1 }

*
R_cam_to_rect *Tr_velo_to_cam;
[X, Y] = meshgrid(5:1:80, -30:1:30);
X = X(
:);
Y = Y(
:);
while (
ischar(lineM)
)
imgName = fullfile(imgDir, [imgPrefix lineM]);
img = imread(imgName);
fig = figure('Position', [20 100
size(img,
2)
size(img,
1)]); axes('Position',[0 0 1 1]);
imshow(img);
hold on;

modelParams = str2num(fgetl(fM));
numModels = modelParams(1);
paramNum = modelParams(2);

for
i = 1
:
numModels
        params = fscanf(fM, '%f', paramNum);
if (
strcmpi(model,
'surface'))
Z = params(1) + params(2) * X + params(3) * Y + params(4) * (X.^ 2) + params(5) * (X.*Y) + params(6) * (Y.^ 2) +
    params(7) * (X.^ 3) + params(8) * ((X.^ 2).*Y) + params(9) * (X.*(Y.^ 2)) + params(10) * (Y.^ 3);

elseif (strcmpi(model,

'plane'))
Z = -(params(1) * X + params(2) * Y + params(4)) / params(3);

elseif (strcmpi(model,

'road'))
Z = params(1) + params(2) * X + params(3) * Y + params(4) * (X.^ 2) + params(5) * (Y.^ 2);

elseif (strcmpi(model,

'line3D'))
t = [-10:
0.1:10]';
X = params(1) + params(4) * t;
Y = params(2) + params(5) * t;
Z = params(3) + params(6) * t;

elseif (strcmpi(model,

'line2D'))
t = [-10:
0.1:10]';
x = 10;
y = -(params(1) * x + params(3)) / params(2);
X = x + params(2) * t;
Y = y - params(1) * t;
Z = -1.68 + 0 * t;

elseif (strcmpi(model,

'spline'))
splineParams = 6;
ycfs = reshape(params(2
:params(1)),[splineParams, (params(1) - 1) / splineParams])';
zcfs = reshape(params((params(1) + 1)
:end),[splineParams, (params(1) - 1) / splineParams])';
X = min(ycfs(
:, 1)):0.1:

max (ycfs(

:, 2));
Y = zeros(size(X));
Z = zeros(size(X));
for
i = 1
:
length(X)
for
r = 1
:
size(ycfs,
1)
t = X(i) - ycfs(r, 1);
if ((
X(i)
>=
ycfs(r,
1) &&
X(i)
<
ycfs(r,
2)))
Y(i) = (ycfs(r, 3) + ycfs(r, 4).*t + ycfs(r, 5).*(t.^ 2) + ycfs(r, 6).*(t.^ 3));
Z(i) = (zcfs(r, 3) + zcfs(r, 4).*t + zcfs(r, 5).*(t.^ 2) + zcfs(r, 6).*(t.^ 3));
break;
end
        end
end
        X = X
';
Y = Y
';
Z = Z
';
end

        lidarPts = [X Y
Z];
velo_img = project(lidarPts, P_velo_to_img);
color = [0, 1, 0];
for
j = 1
:stride:
size(velo_img,
1)

plot (velo_img(j,

1),
velo_img(j,
2), 'o', 'MarkerFaceColor', color, 'MarkerEdgeColor', color);

end

fgetl(fM);

fgetl(fM);

end
saveas(fig, fullfile(outputDir,[outputPrefix

lineM]));

lineM = fgetl(fM);
close
        end