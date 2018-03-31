function [imageData,pointCloudData] = getorthoimage(pcdFilePathOrData,pxielSize,space,startA,endA,startL,endL,axis_x,brightness)
%generate ortho image with tunnel pointcloud data
%
%[imageData,samplePointArray] = getorthoimage(pcdFilePathOrData,pxielSize,space,startA,endA,startL,endL,axis_x,brightness)
%
% INPUT:
% pcdFilePathOrData - 隧道点云数据或者文件路径,为las格式或者xyz文本格式，不可缺省
% pxielSize         - 像素大小，默认0.01m
% space             - 点云数据抽稀间隔，即每隔space个读取一个数据，默认为5
% startA、endA      - 生成图像的角度范围，默认30,60
% startL、endL      - 生成图像的里程范围，默认0,10000
% axis_x            - 隧道点云的前进方向，1表示x或者第一列为前进方向，2表示y或者第二列为前进方向，默认2
% brightness        - 亮度系数，值越大图像越明亮，默认1.3
%
% OUTPUT:
% pointCloudData - ortho point cloud data
% imageData      - ortho image data
%
% This program is for processing odered tunnel point cloud data which is 
% collected through single line profile scanning.Example for faro scanner.
% 
%
% The program is written by Chen Qichao in his period of studying in master
% degree at Tongji University. You can redistribute or modify the program
% for non-commercial use. Any commercial use of this program is forbidden
% except being authorized.
%
% mail : mailboxchen@foxmail.com
% Copyright (C) 2015 - 2018  Tongji University

if ~exist('pxielSize','var')||isempty(pxielSize),pxielSize = 0.01;end
if ~exist('space','var')||isempty(space),space = 5;end
if ~exist('startA','var')||isempty(startA),startA = 0;end
if ~exist('endA','var')||isempty(endA),endA = 360;end
if ~exist('startL','var')||isempty(startL),startL = 0;end
if ~exist('endL','var')||isempty(endL),endL = 10000;end
if ~exist('axis_x','var')||isempty(axis_x),axis_x = 2;end
if ~exist('brightness','var')||isempty(brightness),brightness = 1.3;end

%判断输入的是文件路径还是点云矩阵
[row,col] = size(pcdFilePathOrData);
if row>1&&col>=4
    pointCloudData = pcdFilePathOrData;
elseif row==1
    [path,filename,filetype]=fileparts(pcdFilePathOrData);
    if(filetype=='.las')
        A = LASreadAll(pcdFilePathOrData);
        pointCloudData=[A.x,A.y,A.z,A.intensity];
        savepointcloud2file(pointCloudData,filename,false);
    elseif(filetype=='.xyz')|(filetype=='.txt')
        fid=fopen(pcdFilePathOrData,'r');
        pointCloudData = readpointcloudfile2(pcdFilePathOrData);%读取全部点
    %     pointCloudData =  readpointcloudfile(fid,100000);%读取指定个数点
    else
        error('pcdFilePathOrData is not a correct path!');
        return;
    end
else 
    return;
end

   ScanLineArray = getscanline_faro(pointCloudData(1:space:end,:),axis_x);%按轴变动提取扫描线,faro

  % ScanLineArray = slice2scanlines(pointCloudData(1:space:end,:),1);%按点相邻点间距提取扫描线,sick

    samplePointArray = getsamplepoint(ScanLineArray,startA,endA,startL,endL);%样本区域
    pointCloudData = pointArray2Point(samplePointArray);
%     savepointcloud2file(pointCloudData,filename,false);%存储展开的点云
%     imageData = convertpointcloud2img(pointCloudData,pxielSize);%网格法生成图像
    imageData = convertPD2img(pointCloudData,pxielSize);%半径搜索法生成图像
    imageData = imageData*brightness;
    imwrite(imageData,strcat('orthoimage','.png'));%图像生成
end

function  samplepoint = pointArray2Point(samplePointArray)
%
%将点云展开成平面
      nSample = size(samplePointArray,2);
      nPoint = 0;
      for iSample = 1:nSample,
          %计算点个数,用于初始化
        n = size(samplePointArray(iSample).x,1);
        nPoint =nPoint+n;
      end
      x = zeros(nPoint,1);
      y = zeros(nPoint,1);
      ins = zeros(nPoint,1);
      iPoint = 0;
      for iSample = 1:nSample,
          n = size(samplePointArray(iSample).x,1);
          x(iPoint+1:iPoint+n,1) = samplePointArray(iSample).x;
          l(iPoint+1:iPoint+n,1) = samplePointArray(iSample).l;
          ins(iPoint+1:iPoint+n,1) = samplePointArray(iSample).ins;
          iPoint = iPoint+n;
      end
      samplepoint = [x l zeros(nPoint,1) ins];
%       savepointcloud2file(samplepoint,savefilename,false);
end

function samplePointArray = getsamplepoint(ScanLineArray,startA,endA,startL,endL)
%
%这里以x为隧道前进方向，yoh为扫描线断面
%以圆心垂直向下为起始方向，顺时针为正
%startA、endA为提取点的角度范围，lenght为提取的里程长度
%每1000条扫描线进行一次圆参数更新
    lenght = 0;
    nScanline = size(ScanLineArray,2);
    %x为隧道轴线前进坐标，y为横坐标，h为纵坐标，A为以圆心垂直向下顺时针起算的夹角，r为断面圆半径，l为点沿母线展开的后的长度
    PointSet= struct('x',0,'y',0,'h',0,'ins',0,'A',0,'r',0,'l',0);
    samplePointArray=repmat(PointSet,[1 nScanline]);  
    for iScanline = 1:nScanline
        x = ScanLineArray(iScanline).x;
        y = ScanLineArray(iScanline).y;
        h = ScanLineArray(iScanline).h;
        ins = ScanLineArray(iScanline).ins;
        
%         nPoint = size(x,1);      
%         plot3(x,y,h,'r.');axis equal;hold on;
%         continue;

        if mod(iScanline,1000)||iScanline==1
            para = ransac([y h],'circle',0.02);
            if(isempty(para))
                continue;
            end
            x0 = para(1,1);%断面圆心坐标，对应点云中y、h
            y0 = para(2,1);
            r = para(3,1);
            if iScanline~=1
                lenght0 = norm([x0Pre-x0 y0Pre-y0]);
                lenght = lenght+lenght0;
            end
            x0Pre = x0;
            y0Pre = y0;
        end
        if lenght>=startL&&lenght<=endL
            ;
        else
            continue;
        end 
            px = x;
            py = y;
            ph = h;
            pins = ins;
            A = atan2d(ph-y0,py-x0);%注意是atan2d(Y,X)
            A(A>=-90)=(A(A>=-90)+90);
            A(A<-90&A>=-180)=(A(A<-90&A>=-180)+450);   
            [row,~]=find(A>=startA&A<=endA);
            samplePointArray(iScanline).x = px(row);
            samplePointArray(iScanline).y = py(row);
            samplePointArray(iScanline).h = ph(row);
            samplePointArray(iScanline).ins = pins(row);
            samplePointArray(iScanline).A = A(row);
            samplePointArray(iScanline).r = r;
            samplePointArray(iScanline).l = (A(row)./180).*pi*r;   
%             plot3(px(row),py(row),ph(row),'go');axis equal;hold on;
    end   
end
