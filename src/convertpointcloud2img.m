function imageData = convertpointcloud2img(pointCloudData,pxielSize)
datetime('now','TimeZone','local','Format','HH:mm:ss Z')
%     pointCloudFilePath = 'circle_30d60d_All.xyz';
%     pointCloudData = readpointcloudfile2(pointCloudFilePath);
%     pxielSize = 0.005;
    [width,height,minX,minY,maxX,maxY] = calculatesize(pointCloudData,pxielSize);
    gridArray = gridpoint(pointCloudData,pxielSize);%格网化
    greyImage = idw(gridArray,minX,minY,pxielSize);%插值，比较耗时 
%    img =  imread('qq.png');
%    img = rgb2gray(img);
%    hist =  imhist(img);     
    [imageData,T] = histeq(greyImage);
%     figure,plot((0:255)/255,T);
%     imwrite(J,'dense2.png');%图像生成
datetime('now','TimeZone','local','Format','HH:mm:ss Z')
end

function  normPara = normalizegray(imageArray)
%correct grat value
%对像素格网数据进行稀疏采样归一化处理
%取总量万分之一个像素点进行归一化采样
[row,col] = size(imageArray);
rowNum = ceil(row/100);%行采样个数
colNum = ceil(col/100);
   indexRow= getsampleindex(row,rowNum); 
   indexCol= getsampleindex(col,colNum); 
   maxGray = 0;
    for i = 1:rowNum,
        for m = 1:colNum,
          data =  imageArray{indexRow(i),indexCol(m)};
          if isempty(data),
              continue;
          end
          gary =  max(data(:,4));
          if maxGray<gary,
              maxGray = gary;
          end
        end
    end
    if maxGray==0,
        normPara=1;
    else
        normPara = 1/maxGray;
    end
end


function gridArray = gridpoint(pointCloudData,gridSize)
%generate grid of points
    [width,height,minX,minY,maxX,maxY] = calculatesize(pointCloudData,gridSize);
    widthStripsArray = cut2strips(pointCloudData,width,minX,maxX,gridSize,1);
    gridArray = cell(height,width);
    for i = 1:width,
        widthStrip = widthStripsArray{i};
        heightStripsArray = cut2strips(widthStrip,height,minY,maxY,gridSize,2);
        gridArray(:,i) = heightStripsArray';
    end
end

function imageOut = idw(imageArray,minX,minY,pxielSize)
%inverse distance weighted interpolation
radius = 0.02;%插值半径
nGrid = ceil(radius/pxielSize) - 1;%灰度值受周围nGrid个格网影响
[height,width] = size(imageArray);
imageOut = zeros(height,width);
normPara = normalizegray(imageArray);%归一化近似系数
    for iHeight=1:height,
%         if iHeight==200,
%             a=0;
%         end
        for iWidth=1:width,
            %遍历所有格网
            xLR = iWidth - nGrid;
            yLR = iHeight - nGrid;
            xRB = iWidth + nGrid;
            yRB = iHeight + nGrid;
            if xLR<=0,
                xLR=1;
            end
            if yLR<=0,
                yLR=1;
            end
            if xRB>=width,
                xRB=width;
            end
            if yRB>=height,
                yRB=height;
            end
            pointsArray = imageArray(yLR:yRB,xLR:xRB);
            interX = minX+(iWidth-1)*pxielSize+0.5*pxielSize;%插值中心坐标
            interY = minY+(iHeight-1)*pxielSize+0.5*pxielSize;
            [wPoints,hPoints] = size(pointsArray);
            nPoints=0;
            points = zeros(100,4);%先初始化50个内存，避免后面频繁改变长度
            for m=1:wPoints,
                for n=1:hPoints,
                    %遍历影响范围内的格网点
                    %主要在这里计算耗费时间
                    point = pointsArray{m,n};
                    nPoint = size(point,1);
                    if nPoint==0,
                        continue;
                    end
                    nPointsPre = nPoints;
                    nPoints = nPoints+nPoint;
                    points(nPointsPre+1:nPoints,:) = point;
                end
            end
            if nPoints==0,
                continue;
            end
            weightTotal = 0;
            insOut = 0;
            for i = 1:nPoints,
                x = points(i,1);
                y = points(i,2);
                ins = points(i,4);
                dist = norm([interX-x interY-y]);
                %定权次数越高，插值结果对比度越大，一般高于3次方时结果即趋于稳定
                weight = (pxielSize/dist)^3;
                %等权插值，结果比较模糊
%                 weight = 1;
                insOut = insOut+weight*ins;
                weightTotal = weightTotal+weight;
            end
%             insOut = (insOut/weightTotal)/255;
            insOut = (insOut/weightTotal)*normPara;
            imageOut(iHeight,iWidth) = insOut;
        end
    end
end

function stripsArray = cut2strips(pointData,nStrips,startValue,endValue,pxielSize,type)
%cut point into strips
%type==1, cut by x coordinate;
%type==2, cut by y coordinate;
    stripsArray(1:nStrips) = {[]};
    if isempty(pointData),
        return;
    end
    pointData = sortrows(pointData,type);%按x坐标排序
    nPoint = size(pointData,1);
    valueArray = pointData(:,type);%分割的依据，如按x或者y坐标
    cutStart = startValue;
    cutEnd = startValue + pxielSize;
    iPoint=1;
    value = valueArray(1);
    isEndPoint = false;%是否遍历到最后一个点
    for i = 1:nStrips,%分成nStrips条
        strip = [];
        iStripPoint = 0;
        while value<cutEnd,
            iStripPoint = iStripPoint+1;
            strip(iStripPoint,:) = pointData(iPoint,:);
            if iPoint<nPoint,
                iPoint = iPoint+1;   
                value = valueArray(iPoint);
            else
                isEndPoint = true;
                break;
            end
        end  
        stripsArray(i) = {strip};
        cutStart = cutEnd;
        cutEnd = cutEnd + pxielSize;
        if isEndPoint,
            break;
        end
    end
end

function grey = calculategrey(pointData)
%calculate grey value of each pixel
    grey =0;
    if isempty(pointData),
        return;
    end
    ins = pointData(:,4);
    grey = mean(ins)/225;
    grey = mean(ins)
end

function [width,height,minX,minY,maxX,maxY] = calculatesize(pointCloudData,pxielSize)
%calcullate width and height of inage
xAraay = pointCloudData(:,1);
yArray = pointCloudData(:,2);
minX = min(xAraay);
maxX = max(xAraay);
minY = min(yArray);
maxY = max(yArray);
width =  ceil((maxX - minX)/pxielSize);
height = ceil((maxY - minY)/pxielSize);
end

function index = getsampleindex(nPoint,nSample)
%
%在1~nPoint中随机抽取nSample个不重复对像，返回对象索引号
    index = -ones(nSample,1);
    iSample = 0;
    while iSample<nSample,
        rand0 =  floor(1+(nPoint-1)*rand(1,1));
        isSave = true;
        for i = 1:iSample,          
            if rand0==index(i),
                isSave = false;
                break;
            end
        end
        if isSave,
            iSample = iSample+1;
            index(iSample,1) = rand0;
        end
    end
end

