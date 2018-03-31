function  ScanLineArray = slice2scanlines(pointCloudData,intervalDist)
% extract scanlines from point cloud data
% ScanLineArray = slice2scanlines(pointCloudData,intervalDist)

% The program is written by Chen Qichao in his period of studying in master
% degree at Tongji University. You can redistribute or modify the program
% for non-commercial use. Any commercial use of this program is forbidden
% except being authorized.
%
% mail : mailboxchen@foxmail.com
% Copyright (C) 2015 - 2018  Tongji University

% slice point cloud data according to the maximal distance of two contiguous point
% the result of this process will abtain each scanline
%检测扫描线原则：一，间隔大于5米；二，在间隔图像中是顶点；三，顶点之间的点个数大于一定数量
    nPoint = size(pointCloudData,1);
    prePoint = pointCloudData(1:nPoint-1,1:4);
    nextPoint = pointCloudData(2:nPoint,1:4);    
    dx = (prePoint(1:nPoint-1,1)-nextPoint(1:nPoint-1,1));
    dy = (prePoint(1:nPoint-1,2)-nextPoint(1:nPoint-1,2));
    dh = (prePoint(1:nPoint-1,3)-nextPoint(1:nPoint-1,3));
    ds = sqrt(dx.^2+dy.^2+dh.^2);
    %间隔图像,调试用
%     plot(1:10000,ds(1:10000),'r.');
%     hold on
%     plot(1:10000,ds(1:10000));
    vertexInfoArray = zeros(fix(nPoint/100),2);
    nVertex = 1;
    vertexInfoArray(1,1) = 1;
    vertexInfoArray(1,2) = ds(1);   
    intervalQuantity = 5;%每条扫描线最少点个数，初始默认是5个
%     intervalDist = 5;%扫描线分界点之间的最小距离，一般比道路宽度略小
    nVertexTemp = 1;
    vertexInfoArrayTemp = zeros(20,2);
    vertexInfoArrayTemp(1,1) = 1;
    vertexInfoArrayTemp(1,2) = ds(1);
    for i = 2:(nPoint-2),
        %计算间隔参数intervalQuantity
        %当开始部分点云比较杂乱使，切片可能不精确，此时计算的intervalQuantity可能不合适
        %算法却要改进
        preDs = ds(i-1);
        currentDs = ds(i);
        nextDs = ds(i+1);
        if (preDs<currentDs)&&(nextDs<currentDs)&&currentDs>intervalDist,
            %currentDs是顶点
            preVertexOrder = vertexInfoArrayTemp(nVertexTemp,1);
            preVertexDs = vertexInfoArrayTemp(nVertexTemp,2);
            if (i - preVertexOrder)>=intervalQuantity,
                nVertexTemp = nVertexTemp+1;
                vertexInfoArrayTemp(nVertexTemp,1) = i;
                vertexInfoArrayTemp(nVertexTemp,2) = ds(i);
            elseif ((i - preVertexOrder)<5)&&(preVertexDs<ds(i)),
                vertexInfoArrayTemp(nVertexTemp,1) = i;
                vertexInfoArrayTemp(nVertexTemp,2) = ds(i);
            end
            if nVertexTemp>10,
                break;
            end
        end
    end  
    sumIntervalTemp = 0;
    for i = 1:nVertexTemp-1,
        intervalTemp = vertexInfoArrayTemp(i+1,1) - vertexInfoArrayTemp(i,1);
        sumIntervalTemp = sumIntervalTemp+intervalTemp;
    end
    intervalQuantity = fix((sumIntervalTemp/nVertexTemp)*0.75);%此参数可能不准
%     intervalQuantity = 90;
    for i = 2:(nPoint-2),
        preDs = ds(i-1);
        currentDs = ds(i);
        nextDs = ds(i+1);
        if (preDs<currentDs)&&(nextDs<currentDs)&&(currentDs>intervalDist),
            %currentDs是顶点
            preVertexOrder = vertexInfoArray(nVertex,1);
            preVertexDs = vertexInfoArray(nVertex,2);
            if (i - preVertexOrder)>=intervalQuantity,
                nVertex = nVertex+1;
                vertexInfoArray(nVertex,1) = i;
                vertexInfoArray(nVertex,2) = ds(i);
            elseif ((i - preVertexOrder)<intervalQuantity)&&(preVertexDs<ds(i)),
                vertexInfoArray(nVertex,1) = i;
                vertexInfoArray(nVertex,2) = ds(i);
            end
        end     
    end
    nScanLine = nVertex;
    PointSet= struct('x',0,'y',0,'h',0,'ins',0);
    ScanLineArray=repmat(PointSet,[1 nScanLine]);  
    
    for i = 1:nVertex-1,
        nStart = vertexInfoArray(i)+1;%比如有5多个点，但只有4个间距
        nEnd = vertexInfoArray(i+1);
        ScanLineArray(i).x=pointCloudData(nStart:nEnd,1);
        ScanLineArray(i).y=pointCloudData(nStart:nEnd,2);
        ScanLineArray(i).h=pointCloudData(nStart:nEnd,3);
        ScanLineArray(i).ins=pointCloudData(nStart:nEnd,4);
    end
    if nPoint>nEnd,
        %当还有剩余点没有被切割成扫描线时，将剩余的点归为一条扫描线
        ScanLineArray(nVertex).x=pointCloudData(nEnd+1:nPoint,1);
        ScanLineArray(nVertex).y=pointCloudData(nEnd+1:nPoint,2);
        ScanLineArray(nVertex).h=pointCloudData(nEnd+1:nPoint,3);
        ScanLineArray(nVertex).ins=pointCloudData(nEnd+1:nPoint,4);
    else
        ScanLineArray = ScanLineArray(1:nVertex-1);
    end
end