clc;clear all; close all; dbstop if error;
addpath(' /home/binbin/disk/project-code/lane_quality_assessment/ICNet-master/LaneRecognitionTracking/matlab/bspline');
%%%%%%%%%%%%%%%%%%%%%%%%%

%%0056
% vioPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0056/Lanes/vio.txt';
% worldCenterPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0056/Lanes/worldCenterCoordinates.txt';
% worldLeftPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0056/Lanes/worldLeftCoordinates.txt';
% worldRightPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0056/Lanes/worldRightCoordinates.txt';

orginalGPS = [49.0283797	8.35762004
49.0283825	8.35760696
49.0283853	8.35759388
49.028388	8.3575808
49.0283908	8.35756772
49.0283936	8.35755464
49.0283964	8.35754156
49.0283991	8.35752848
49.0284019	8.3575154
49.0284047	8.35750233
49.0284074	8.35748925
49.0284102	8.35747617
49.028413	8.35746309
49.0284157	8.35745001
49.0284185	8.35743693
49.0284213	8.35742385
49.0284241	8.35741077
49.0284268	8.35739769
49.0284296	8.35738461
49.0284324	8.35737153
49.0284351	8.35735845
49.0284379	8.35734538
49.0284407	8.3573323
49.0284434	8.35731922
49.0284462	8.35730614
49.028449	8.35729306
49.0284518	8.35727998
49.0284545	8.3572669
49.0284573	8.35725382
49.0284601	8.35724074
49.0284628	8.35722766
49.0284656	8.35721458
49.0284684	8.3572015
49.0284711	8.35718843
49.0284739	8.35717535
49.0284767	8.35716227
49.0284795	8.35714919
49.0284822	8.35713611
49.028485	8.35712303
49.0284878	8.35710995
49.0284905	8.35709687
49.0284933	8.35708379
49.0284961	8.35707071
49.0284988	8.35705763
49.0285016	8.35704455
49.0285044	8.35703148
49.0285072	8.3570184
49.0285099	8.35700532
49.0285127	8.35699224
49.0285155	8.35697916
49.0285173, 8.3569756
49.0285182	8.35696608
49.028521	8.356953
49.028521	8.356953
49.028524	8.356934
49.028524	8.356934
49.028527	8.356917
49.02853	8.3569
49.02853	8.3569
49.028519	8.3568935
49.028508	8.356887
49.028508	8.356887
];

gtgps = [49.028379740816 8.3576200380035
49.028381626571 8.357611579154 
49.028383714571 8.3576022613723
49.028385629575 8.3575938114421
49.028387555844 8.3575853346817
49.028389671629 8.3575760371257
49.028391606176 8.3575675559024
49.028393748009 8.3575582682869
49.028395691785 8.3575497980175
49.028397609303 8.3575413345818
49.028399510054 8.3575328887407
49.02840159795 8.3575236062221 
49.02840349241 8.3575152095749 
49.028405374591 8.3575068133665
49.028407418702 8.3574976078991
49.028409264446 8.3574892318202
49.028411099791 8.3574808761613
49.028413113034 8.3574716886406
49.028414933284 8.3574633462768
49.028416755206 8.3574550297427
49.028418741098 8.3574458709042
49.028420533118 8.3574375451527
49.02842232199 8.357429219452  
49.028424297365 8.3574200745933
49.028426079264 8.3574117792149
49.028428030776 8.3574026606216
49.028429803641 8.3573943704334
49.028431573415 8.3573860847493
49.028433500375 8.3573769731287
49.028435246223 8.3573686834151
49.028436997123 8.3573604034788
49.028438926269 8.3573513200601
49.028440684233 8.3573430896831
49.028442439838 8.3573348761216
49.028444365605 8.3573258705808
49.028446111298 8.3573177139922
49.028447861404 8.3573095899878
49.028449798685 8.3573006948093
49.02845155859 8.3572926178987 
49.028453324523 8.3572845719629
49.028455264608 8.3572757371141
49.028457023208 8.3572677319311
49.028458791006 8.3572597406817
49.028460770485 8.3572510153537
49.028462589014 8.3572431104773
49.028464419229 8.3572352653346
49.028466441759 8.3572266802513
49.028468305271 8.3572189490855
49.028470174707 8.3572112703278
49.028472229078 8.3572028883621
49.028474092956 8.3571953630785
49.02847614863 8.3571872075518 
49.028478018559 8.3571799145532
49.028479866312 8.357172727692 
49.02848189403 8.3571649851989 
49.028483722037 8.3571580840148
49.028485534958 8.3571513396649
49.028487499181 8.3571440924839
49.028489272197 8.3571376972127
49.028491027269 8.3571314767143
49.028492931268 8.3571248571717
49.028494632415 8.3571190402135
49.028496294753 8.3571134183462
49.02849808633 8.3571074760533 
49.02849966696 8.3571022791142 
49.028501380794 8.3570967739278
49.028502913029 8.3570919232997
49.02850440336 8.3570871980066 
49.028506008473 8.3570821615701
49.028507416837 8.3570777467623
49.028508785504 8.3570734942185
49.028510090297 8.3570694122307
49.028511473277 8.3570651432593
49.028512645698 8.3570614371321
49.028513871798 8.3570575931078
49.028514900388 8.3570542909816
49.028515862535 8.3570512152278
49.028516839541 8.3570481082241
49.028517648761 8.3570455077314
49.028518375824 8.357043110757 
49.028519081685 8.3570407358669
49.028519648476 8.3570387909804
49.028520163672 8.3570370555475
49.028520697021 8.3570352051159
49.028521146996 8.3570335241398
49.028521587315 8.3570318367759
49.028522074361 8.3570300349466
49.02852250197 8.3570283707202 
49.028522910289 8.3570266973063
49.028523339821 8.3570248632845
49.028523701839 8.3570231816585
49.028524049297 8.3570214931161
49.028524424198 8.3570195775451
49.028524765422 8.3570177925886
49.028525098606 8.3570158872317
49.028525458586 8.3570136475797
49.028525775069 8.357011459805 
49.028526107369 8.357008887632 
49.028526393051 8.3570063814276
49.028526661049 8.35700372498  
49.028526924356 8.3570006023795
49.028527128549 8.3569975838924
49.028527283359 8.3569943655999
49.028527387983 8.3569905928328
49.028527429539 8.356986911436 
49.028527416452 8.35698298819  
49.028527319123 8.3569783981246
49.028527108183 8.3569739745429
49.028526752781 8.3569693448028
49.028526176417 8.3569640410259
49.02852549096 8.356959125336  
49.028524559234 8.3569536930641
49.028523575635 8.3569487509482
49.02852245641 8.3569438238653 
49.02852119884 8.3569388991306 
49.028519638741 8.3569335393656
49.028518044457 8.3569287157615
49.028516117186 8.3569234857478
49.028514212284 8.3569187748938
49.028512167863 8.3569141295178
49.028509729456 8.356909141209 
49.028507359186 8.3569046956496
];
%0035
vioPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0035/0035_Lanes/2011_09_26_0035-vio.txt';
worldCenterPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0035/0035_Lanes/worldCenterCoordinates1.txt';
worldLeftPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0035/0035_Lanes/worldLeftCoordinates.txt';
worldRightPath = '/home/binbin/disk/Dropbox/KITTI/2011_09_26_0035/0035_Lanes/worldRightCoordinates.txt';


% compute length
slamat = readtable(vioPath);
RotTransMat = table2cell(slamat);
RotTransMat = cell2mat(RotTransMat(:, 1:12));
tranjectory = 0;
trans = [RotTransMat(:, 12), RotTransMat(:, 4), RotTransMat(:, 8)];

%display lbs and lcc
LccCtrlPts = readtable(worldCenterPath);
LccCtrlPtsMat = table2array(LccCtrlPts);
LccCtrlPtsMat(:, 2)  =  - LccCtrlPtsMat(:, 2);

LeftCtrlPts = readtable(worldLeftPath);
LeftCtrlPtsMat = table2array(LeftCtrlPts);
LeftCtrlPtsMat(:, 2)  =  - LeftCtrlPtsMat(:, 2);

RightCtrlPts = readtable(worldRightPath);
RightCtrlPtsMat = table2array(RightCtrlPts);
RightCtrlPtsMat(:, 2)  =  - RightCtrlPtsMat(:, 2);

numfCtrlPts = min(size(LccCtrlPtsMat, 1), min(size(LeftCtrlPtsMat, 1), size(RightCtrlPtsMat, 1)));

% get the spline curve
order = 5;
ptselect = 1:10: numfCtrlPts;
LccCtrlPtSpline = computeSpline(order, ptselect, numfCtrlPts, LccCtrlPtsMat);
LeftCtrlPtsSpline = computeSpline(order, ptselect, numfCtrlPts, LeftCtrlPtsMat);
RightCtrlPtSpline = computeSpline(order, ptselect, numfCtrlPts, RightCtrlPtsMat);


% % figure; hold on; grid on; box on;
% % xlabel('x'); ylabel('y');
% % set(gca,'FontSize',24);
% % plot(LccCtrlPtSpline(1,:), LccCtrlPtSpline(2,:), 'b');
% % plot(LeftCtrlPtsSpline(1,:), LeftCtrlPtsSpline(2,:),  'g');
% % plot(RightCtrlPtSpline(1,:), RightCtrlPtSpline(2,:), 'g');

latitude1 = 49.028379740816;
longitude1 = 8.3576200380035;
[latitude3,longitude3] = coordinateTocoordinate(latitude1,longitude1,LccCtrlPtsMat(:, 1:2)');
gps_data_interp = [latitude3, longitude3];

% xq = linspace(min((gps_data_interp(1:end, 1))), (max(gps_data_interp(1: end , 1))),  70);
% pointset = unique(gps_data_interp, 'rows');
% yq = interp1(pointset(1:end, 1),pointset(1:end, 2),xq, 'spline');
ptselect = 1:5: size(gps_data_interp, 1);
newxyspline = computeSpline(5,ptselect, numfCtrlPts,  gps_data_interp);

ptselect = 1:5: size(gtgps, 1);
knots = [0, 0, 0, 0, linspace(0, 1, size(ptselect, 2) -2),  1, 1, 1];
C = bspline_deboor(5,knots,gtgps(ptselect, :)');
[dt,dP] = bspline_deriv(5,knots,gtgps(ptselect, 1:2)');
dC = bspline_deboor(4,dt,dP);

ptselect = 1:5: size(dC, 2);
knots = [0, 0, 0, 0, linspace(0, 1, size(ptselect, 2) -2),  1, 1, 1];
C = bspline_deboor(5,knots,dC(:, ptselect));
[dt,dP] = bspline_deriv(5,knots,dC(1:2, ptselect));
d2C = bspline_deboor(4,dt,dP);


newxyspline = newxyspline';
ptselect = 1:5: size(newxyspline, 1);
knots = [0, 0, 0, 0, linspace(0, 1, size(ptselect, 2) -2),  1, 1, 1];
C = bspline_deboor(5,knots,newxyspline(ptselect', :)');
[dt,dP] = bspline_deriv(5,knots,newxyspline(ptselect, 1:2)');
dn = bspline_deboor(4,dt,dP);

ptselect = 1:5: size(dn, 2);
knots = [0, 0, 0, 0, linspace(0, 1, size(ptselect, 2) -2),  1, 1, 1];
C = bspline_deboor(5,knots,dn(:, ptselect));
[dt,dP] = bspline_deriv(5,knots,dn(1:2, ptselect));
d2n = bspline_deboor(4,dt,dP);

smallv = 10000000000;
vec = [];
vec2 = [];
for i = 1 : size(dC, 2)
    for j = 1 : size(dn, 2)
            if(abs(dC(1, i) - dn(1, j)) < smallv)
                smallv = abs(dC(1, i) - dn(1, j));
                nearx = dn(1, j);
                neary = dn(2, j);
            end
    end
    vec = [vec; abs(dC(1, i)) ,abs(dC(2, i)) , nearx ,neary ,smallv];
end

for i = 1 : size(d2C, 2)
    for j = 1 : size(d2n, 2)
            if(abs(d2C(1, i) - d2n(1, j)) < smallv)
                smallv = abs(d2C(1, i) - d2n(1, j));
                nearx = d2n(1, j);
                neary = d2n(2, j);
            end
    end
    vec2 = [vec2; abs(d2C(1, i)) ,abs(d2C(2, i)) , nearx ,neary ,smallv];
end

figure; hold on;
plot(1:size(vec, 1), abs(vec(:, 2)), 'r*-');
plot(1:size(vec, 1), abs(vec(:, 4)), 'bs-');

% plot(1:size(vec2, 1), abs(vec2(:, 2)), 'm*-');
% plot(1:size(vec2, 1), abs(vec2(:, 4)), 'ys-');

figure; hold on;
plot(orginalGPS(:, 1), orginalGPS(:, 2), 'b*');
plot(gps_data_interp(:, 1), gps_data_interp(:, 2), 'g*');
plot(gtgps(:, 1), gtgps(:, 2), 'rs');
plot(newxyspline(1, :), newxyspline(2, :), 'ms-');


dlmwrite('//home/binbin/Downloads/gps_35_interp.csv',gps_data_interp,'precision', 9);

function LccSplinexy = computeSpline(order,ptselect,  numfCtrlPts, CtrlPtSpline)
knots = [0, 0, 0, 0, linspace(0, 1, size(ptselect, 2) -2),  1, 1, 1];
LccSplinexy = bspline_deboor(order, knots, CtrlPtSpline(ptselect, 1:2)');
end

function dist = CoordinatesToMeters(latitude1,longitude1,latitude2,longitude2)

m_earthDiameterMeters = 6365.998 * 2 * 1000;
latitude1  = deg2rad(latitude1);
longitude1 = deg2rad(longitude1);
latitude2  = deg2rad(latitude2);
longitude2 = deg2rad(longitude2);

x = sin((latitude2 - latitude1)/2);
y = sin((longitude2 -longitude1)/2);

dist =  m_earthDiameterMeters * asin(sqrt((x*x) + (cos(latitude1)*cos(latitude2)*y*y)));
end

function [degree meter] = coordinatestodegreeangle(pts1, pts2)
%pts1 pts2 are 1 x 2 row vector
meter =- sqrt((pts1(:, 1) -  pts2(:, 1))^2  +  (pts1(:, 2)  -  pts2(:, 2))^2);
degree = tan( (pts2(:, 2)  -  pts1(:, 2))/ (pts2(:, 1)  -  pts1(:, 1)) );
if degree < 0
    degree = abs(rad2deg(degree)) + 115;
else
    degree = 115 - abs(rad2deg(degree)) ;
end
end

function [latitude3,longitude3] = coordinateTocoordinate(latitude1,longitude1,curvePts)
% curvePts n x 2
curvePts = curvePts';
degreeMat = [];
meterMat = [];
latitude1 = deg2rad(latitude1);
longitude1 = deg2rad(longitude1);
for i = 2 : size(curvePts, 1)
    [degree, meter] = coordinatestodegreeangle(curvePts(i, :), curvePts(1, :));
    degreeMat = [degreeMat; degree];
    meterMat = [meterMat; meter];
end
anglerad = deg2rad(degreeMat);
metersearth =meterMat * 2 / (6365.998 * 2 * 1000);
latitude3 = asin((sin(latitude1) * cos(metersearth)))+ (cos(latitude1) * sin(metersearth) .* cos(anglerad));
longitude3 = longitude1 + atan2((sin(anglerad) .* sin(metersearth) * cos(latitude1)), cos(metersearth) - (sin(latitude1) * sin(latitude3)));
latitude3 = rad2deg(latitude3);
longitude3 = rad2deg(longitude3);
end


