function ScanLineArray = getscanline_faro(pointCloudData,axis_x)
%extract scanlines from tunnel pointcloud data coleected by faro scanner
% ScanLineArray = getscanline_faro(pointCloudData,axis_x)
%
% The program is written by Chen Qichao in his period of studying in master
% degree at Tongji University. You can redistribute or modify the program
% for non-commercial use. Any commercial use of this program is forbidden
% except being authorized.
%
% mail : mailboxchen@foxmail.com
% Copyright (C) 2015 - 2018  Tongji University

%认为同一条扫描线x坐标不变化，以x来提取扫描线,适合Faro扫描的数据

x = pointCloudData(:,axis_x);
axis_y=2;
if(axis_x==2)
    axis_y=1;
end
nPoint = size(x,1);
scanlineX = x(1,1);
scanLineIndex(1,1) = 1;
nScanline = 1;
for iPoint = 1:nPoint,
    pointX = x(iPoint,1);
    if pointX==scanlineX,
        scanLineIndex(nScanline,2) = iPoint;
    else
        scanlineX = pointX;
        nScanline = nScanline+1;
        scanLineIndex(nScanline,1) = iPoint;
    end
end
PointSet= struct('x',0,'y',0,'h',0,'ins',0);
ScanLineArray=repmat(PointSet,[1 nScanline]);
for iScanline = 1:nScanline
    startIndex = scanLineIndex(iScanline,1);
    endIndex = scanLineIndex(iScanline,2);
    ScanLineArray(iScanline).x = pointCloudData(startIndex:endIndex,axis_x);
    ScanLineArray(iScanline).y = pointCloudData(startIndex:endIndex,axis_y);
    ScanLineArray(iScanline).h = pointCloudData(startIndex:endIndex,3);
    ScanLineArray(iScanline).ins = pointCloudData(startIndex:endIndex,4);
end
end