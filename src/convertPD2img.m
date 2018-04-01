function [imageData,gridArray] = convertPD2img(pointData,pxielSize,radius,isRotate)
% convert pointcloud data into raster intensity image，a substitutor of convertpointcloud2img
% [imageData,gridArray] = convertPD2img(pointData,pxielSize,radius,isRotate)
%
% arguments: (input)
% radius - 像素插值半径，过大会增加计算量，过小图像会产生黑洞像素点，一般设置
%          为点的间隔大小
% isRotate - （OPTIONAL）- 是否旋转图像，通过旋转可以使道路图像延轨迹水平放置，
%           避免图像与图幅有较大夹角
%
% arguments: (output)
% radius - 像素插值半径，过大会增加计算量，过小图像会产生黑洞像素点，一般设置
%          为点的间隔大小
%           DEFAULT: 'radius'    (pxielSize*3)
%                    'isRotate'  (true)
%
% isRotate - （OPTIONAL）- 是否旋转图像，通过旋转可以使道路图像延轨迹水平放置，
%           避免图像与图幅有较大夹角
% 
% arguments: (output)
% imageData - 转化后的灰度图像
% gridArray - 灰度图像每个像素对应的点云
% 

% The program is written by Chen Qichao in his period of studying in master
% degree at Tongji University. You can redistribute or modify the program
% for non-commercial use. Any commercial use of this program is forbidden
% except being authorized.
%
% mail : mailboxchen@foxmail.com
% Copyright (C) 2015 - 2018  Tongji University

% datetime('now','TimeZone','local','Format','HH:mm:ss Z')
isRotate=false;
if ~exist('radius','var')||isempty(radius),radius = pxielSize*2;end
if ~exist('isRotate','var')||(isRotate==true)
    x = pointData(:,1);
    y = pointData(:,2);
    [rectx,recty,~,~] = minboundrect(x,y);
    d = sqrt((rectx(1:2) - rectx(2:3)).^2+(recty(1:2) - recty(2:3)).^2);%外接矩形边长
    [a,idx_a] = max(d);%较长的边
    b = min(d);
    rotateA = atand((recty(idx_a)-recty(idx_a+1))/(rectx(idx_a)-rectx(idx_a+1)));%外接矩形较长的边于x轴夹角
    if rotateA>=0
        [minY,idx_minY] = min(recty);
        origin  = [rectx(idx_minY) minY];%图像原点对应坐标
    else
        [minX,idx_minX] = min(rectx);
        origin  = [minX recty(idx_minX)];%图像原点对应坐标
    end
else
    [width,height,minX,minY,maxX,maxY] = calculatesize(pointData,pxielSize);
    a = maxX - minX;
    b = maxY - minY;
    origin = [minX,minY];
    rotateA = 0;
end
%     gridArray = gridpoint(pointData,pxielSize,origin,rotateA);%格网化
    greyImage = idwpartition(pointData,origin,a,b,rotateA,pxielSize,radius);%插值，比较耗时 
    imageData = greyImage;
end

% function gridArray = gridpoint(pointData,gridSize,origin,rotateA)
% % generate grid of points, not recommended
%    % [width,height,minX,minY,maxX,maxY] = calculatesize(pointData,gridSize);
%     
%     x0 = pointData(:,1);
%     y0 = pointData(:,2);
%     if abs(tand(rotateA))~=inf
%         k = tand(rotateA);
%         A = k;
%         B = -1;
%         C = origin(2)-k*origin(1);
%         d1 = abs(A.*x0+B.*y0+C)./sqrt(A*A+B*B);%点到长边的距离,对应的是Y
%         %点到短边的距离，对应旋转后的x距离
%         k = tand(rotateA+90);
%         A = k;
%         B = -1;
%         C = origin(2)-k*origin(1);
%         d2 = abs(A.*x0+B.*y0+C)./sqrt(A*A+B*B);
%     else
%         d1 = y0-origin(2);
%         d2 = x0 - origin(1);
%     end
%     minX = min(d2);%d1.d2类似于x，y
%     minY = min(d1);
%     maxX = max(d2);
%     maxY = max(d1);
%     width = ceil((maxX-minX)/gridSize);
%     height = ceil((maxY-minY)/gridSize);
%     pointData(:,5) = d2;%将切割条件放在5,6位，如果5,6位存储了其他数据，这里就得修改到其他位
%     pointData(:,6) = d1;
%     widthStripsArray = cut2strips(pointData,width,minX,maxX,gridSize,5);
%     gridArray = cell(height,width);
%     for i = 1:width
%         widthStrip = widthStripsArray{i};
%         heightStripsArray = cut2strips(widthStrip,height,minY,maxY,gridSize,6);
%         gridArray(:,i) = heightStripsArray';
%     end
% end
% 
% function stripsArray = cut2strips(pointData,nStrips,startValue,endValue,pxielSize,type)
% %cut point into strips
% %type==1, cut by x coordinate;
% %type==2, cut by y coordinate;
% %type也可以是其他指定列;
%     stripsArray(1:nStrips) = {[]};
%     if isempty(pointData)
%         return;
%     end
%     pointData = sortrows(pointData,type);%按x坐标排序
%     nPoint = size(pointData,1);
%     valueArray = pointData(:,type);%分割的依据，如按x或者y坐标
%     cutStart = startValue;
%     cutEnd = startValue + pxielSize;
%     iPoint=1;
%     value = valueArray(1);
%     isEndPoint = false;%是否遍历到最后一个点
%     for i = 1:nStrips,%分成nStrips条
%         strip = [];
%         iStripPoint = 0;
%         while value<cutEnd,
%             iStripPoint = iStripPoint+1;
%             strip(iStripPoint,:) = pointData(iPoint,:);
%             if iPoint<nPoint,
%                 iPoint = iPoint+1;   
%                 value = valueArray(iPoint);
%             else
%                 isEndPoint = true;
%                 break;
%             end
%         end  
%         stripsArray(i) = {strip};
%         cutStart = cutEnd;
%         cutEnd = cutEnd + pxielSize;
%         if isEndPoint,
%             break;
%         end
%     end
% end


function [width,height,minX,minY,maxX,maxY] = calculatesize(pointCloudData,pxielSize)
%calcullate width and height of image
xAraay = pointCloudData(:,1);
yArray = pointCloudData(:,2);
minX = min(xAraay);
maxX = max(xAraay);
minY = min(yArray);
maxY = max(yArray);
width =  ceil((maxX - minX)/pxielSize);
height = ceil((maxY - minY)/pxielSize);
end

function [imageOut,gridArray]= idwpartition(pointData,origin,a,b,rotateCloudA,pxielSize,radius)
%  partition processing for pointcloud by inverse distance weighted interpolation 
%
% arguments(input):
% pointData - 点云数据xyzi
% origin - 插值矩形原点（左下角）
% a - 插值矩形宽
% b - 插值矩形高
% rotateCloudA - 原坐标系到插值矩形坐标系旋转角（顺时针为正）
% radius - 0.10;插值半径，可以填补空洞像素点
%
% 插值矩形指对点云的插值范围，一般有两种，一种是与点云坐标系xy轴平行的外接矩形，
% 另一种是最小外接矩形。如果是最小外接矩形，则a对应长边，b对应短边，因为道路是
% 线状的，希望插值后的图像是左右走向，而非上下走向。
%
% arguments(output):
% imageOut - 插值后图像
% gridArray - 插值图像每个像素对应的点云，暂缺
%
imageOut = [];
np = size(pointData,1);
if np<1000000&&np>0
    partN = 1;
elseif np<=10000000
    partN = ceil(np/1000000);
elseif np>10000000
    partN = 10;
else
    return;
end
    
maxI = max(pointData(:,4));
minI = min(pointData(:,4));
minX = origin(1);
minY = origin(2);
height = ceil(b/pxielSize);
width = ceil(a/pxielSize);
% imageOut2 = zeros(height,width);
% normPara = normalizegray(imageArray);%归一化近似系数

interX = (0.5*pxielSize:pxielSize:width*pxielSize);%插值中心米制像素坐标
interY = (0.5*pxielSize:pxielSize:height*pxielSize)';

dx = (max(interX) - min(interX))/partN;% 分割间隔
dy = (max(interY) - min(interY))/partN;% 分割间隔

if height>width
    d = dy;
    index = 2;
     minO = minY;
     partNa = 1;
    partNb = partN;
else
    d = dx;
    index = 1;
    minO = minX;
    partNa = partN
    partNb = 1;
end

for i =1:partN
%     datetime
    seg1 = minO +(i-1)*d-radius;
    seg2 = minO +i*d+radius;
    segPoint = pointData((pointData(:,index)>seg1)&(pointData(:,index)<seg2),:);
    imagetmp= idw(segPoint,origin,a/partNa,b/partNb,rotateCloudA,pxielSize,radius,maxI,minI);
    origin(index) = origin(index)+d;
    if  height<width
        imageOut = [imageOut imagetmp];
    else
        imageOut = [imageOut;imagetmp];
    end  
end

end

function [imageOut,gridArray]= idw(pointData,origin,a,b,rotateCloudA,pxielSize,radius,maxI,minI)
%inverse distance weighted interpolation for pointcloud
%
% arguments(input):
% pointData - 点云数据xyzi
% origin - 插值矩形原点（左下角）
% a - 插值矩形宽
% b - 插值矩形高
% rotateCloudA - 原坐标系到插值矩形坐标系旋转角（顺时针为正）
% radius - 0.10;插值半径，可以填补空洞像素点
%
% 插值矩形指对点云的插值范围，一般有两种，一种是与点云坐标系xy轴平行的外接矩形，
% 另一种是最小外接矩形。如果是最小外接矩形，则a对应长边，b对应短边，因为道路是
% 线状的，希望插值后的图像是左右走向，而非上下走向。
%
% arguments(output):
% imageOut - 插值后图像
% gridArray - 插值图像每个像素对应的点云，暂缺
%

Mdl = KDTreeSearcher(pointData(:,1:2));%建立kd搜索树
% maxI = max(pointData(:,4));
% minI = min(pointData(:,4));
minX = origin(1);
minY = origin(2);
height = ceil(b/pxielSize);
width = ceil(a/pxielSize);
imageOut = zeros(height,width);
% normPara = normalizegray(imageArray);%归一化近似系数
if maxI~=minI
    normPara = 1/abs(1*maxI-minI);%大于0.8maxI的像素点会
else
    normPara = 1;
end
interX = (0.5*pxielSize:pxielSize:width*pxielSize);%插值中心米制像素坐标
interY = (0.5*pxielSize:pxielSize:height*pxielSize)';
interX = repmat(interX,height,1);
interY = repmat(interY,1,width);
rotateImageA = atand(interY./interX);%插值点在米制像素坐标系中x轴夹角
rotateA = rotateCloudA + rotateImageA;
distO = sqrt(interX.^2+interY.^2);%插值点距米制像素坐标系原点距离
dx = distO.*cosd(rotateA);
dy = distO.*sind(rotateA);
interX = minX + dx;
interY = minY + dy;

ix = reshape(interX',[1 width*height])';
iy = reshape(interY',[1 width*height])';
Idx = rangesearch(Mdl,[ix iy],radius);
for iHeight=1:height
    for iWidth=1:width
        idx_pixel = (iHeight-1)*width+iWidth;%像素点在列向量中的顺序号
        points = pointData(Idx{idx_pixel},:);%插值半径内的点
        nPoints = size(points,1);
        distC = sqrt((points(:,1)-ix(idx_pixel)).^2 + (points(:,2)-iy(idx_pixel)).^2);
        weight = zeros(nPoints,1);
        weight(distC(:,1)~=0,1) = (pxielSize./distC).^3;
        weight(distC(:,1)==0,1) = 1;
        ins = points(:,4);
        insOutTotal = sum(weight.*(ins-minI));
        weightTotal = sum(weight);
        insOut = ((insOutTotal)/weightTotal)*normPara;
        imageOut(iHeight,iWidth) = insOut;
    end
end
end

