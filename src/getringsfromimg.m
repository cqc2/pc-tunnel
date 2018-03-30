function [rings_locs,rings_wdith,rings_num]=getringsfromimg(I)
% 计算环片数目和坐标
% I - 环片栅格图像
%
% OUTPUT：
% rings_locs - 环缝位置对应像素横坐标
% rings_wdith - 包括两列，第一列为环片像素宽度，第二列为标记量，0表示正常，1表
%               示对应环缝附近可能有错检或者漏检
% rings_num - 检测出的环缝个数

I = im2double(I);
[H1,H2] = gradienthist(I);
data=H2;%使用H2检测效果较好

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 顶点检测参数
Fs =1; %采样频率
MinPeakProminence = max(data)/1; %峰最小突起幅度门限
threshold = 0; %峰值点与邻近点比较门限
MinPeakHeight = max(data)/5; %最小峰高度门限
MinPeakDistance = 40/Fs; %最小峰间距门限，大概值
nPeaks = 90000; %最多找nPeaks个峰
sortstr = 'none'; %结果排序
Annotate = 'extents'; 
%峰宽度计算标准,halfprom:半突起幅度宽； halfheight:半高宽
WidthReference = 'halfprom';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%确定MinPeakDistance
[pks,locs,w,p] = ...
    findpeaks(data,Fs,'MinPeakProminence',MinPeakProminence, ...
'threshold',threshold,'MinPeakHeight',MinPeakHeight, ...
'MinPeakDistance',MinPeakDistance,'npeaks',nPeaks, ...
'sortstr',sortstr, ...
'Annotate',Annotate,'WidthReference',WidthReference);
dd=[(locs(2:end)-locs(1:end-1))';190;199;100];
pren = size(dd,1);
nextn=0;
while(pren~=nextn)
    md = mean(dd);
    derta = sqrt(sum((dd-md).^2)/size(dd,1));
    dd = dd(abs(dd-md)<2*derta); 
    pren = nextn;
    nextn = size(dd,1);
end
MinPeakDistance = mean(dd)*0.75;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%接缝位置识别
[pks1,locs1,w1,p1] = ...
    findpeaks(data,Fs,'MinPeakProminence',MinPeakProminence, ...
'threshold',threshold,'MinPeakHeight',MinPeakHeight, ...
'MinPeakDistance',MinPeakDistance,'npeaks',nPeaks, ...
'sortstr',sortstr, ...
'Annotate',Annotate,'WidthReference',WidthReference);

 [pks2,locs2,w2,p2] =...
     findpeaks(-data,Fs,'MinPeakProminence',MinPeakProminence, ...
'threshold',threshold,'MinPeakHeight',MinPeakHeight, ...
'MinPeakDistance',MinPeakDistance,'npeaks',nPeaks, ...
'sortstr',sortstr, ...
'Annotate',Annotate,'WidthReference',WidthReference);

locs1 =locs1';
locs2 = locs2';
for(i=1:size(locs1,1))
    dist = abs(locs2-locs1(i,1));
    [A,idx] = sortrows(dist);
    if(A(1,1)<MinPeakDistance*0.1)
        locs_result(i,1) = ceil((locs1(i,1)+locs2(idx(1),1))/2);
    else
        locs_result(i,1) = locs1(i,1);
    end
end

rings_num = size(locs_result,1);%环片数目
rings_locs = locs_result;%环片位置
ring_wdith = [rings_locs] - [0;rings_locs(1:end-1)];
tmp = ring_wdith;
pren = size(tmp,1);
nextn=0;
while(pren~=nextn)
    md = mean(tmp);
    derta_tmp = sqrt(sum((tmp-md).^2)/size(tmp,1));
    tmp = tmp(abs(tmp-md)<2*derta_tmp); 
    pren = nextn;
    nextn = size(tmp,1);
end
m_wdith = ceil(mean(tmp));
rings_wdith = [ring_wdith,zeros(size(ring_wdith,1),1)];
rings_wdith(abs(ring_wdith(2:end-1,1)-m_wdith)>10*derta_tmp,2) = 1;%标记可能的漏检、错检异常值

%环缝位置写入图像
position = [];
value = [];
for i= 1:size(ring_wdith,1)
    I(:,abs(ceil(rings_locs(i)))) = 1;
    position =  [position;ceil(rings_locs(i))+5 5];
    value = [value,i];
    if(rings_wdith(i,2)==0)
        box_color(i) = {'green'};
    elseif(rings_wdith(i,2)==1)
        box_color(i) = {'red'};
    end
end
RGB = insertText(I,position,value,'FontSize',30,'BoxColor',box_color);
imwrite(RGB,'result_rings.png');%图像生成
end


function [H,H2] = gradienthist(I)
%
%按像素横坐标计算梯度直方图
[FX,FY] = gradient(I);
[row col] = size(FX); 
H = zeros(1,col);%梯度数值统计
H2 = zeros(1,col);%梯度符号统计
for iCol = 1:col,
    h = 0;
    h2 = 0;
    for iRow = 1:row,
        h = h+FX(iRow,iCol); 
        if FX(iRow,iCol)>0;
            h2=h2+1;
        elseif FX(iRow,iCol)<-0;
            h2=h2-1;
        end
    end
    H(iCol) = h;
    H2(iCol) = h2;
end
% A2=fspecial('gaussian',2,5);     
% H2 = filter2(A2,H);
% for i = 1:iCol,
%     if abs(H(i))<1,
%         H(i)=0;
%     end
% end
% plot(1:col,H,'r-');hold on
% plot(1:col,H2,'b-');


% Igrad = sqrt(FY.^2+FX.^2);
% [Igrad,T] = histeq(Igrad);
% imwrite(Igrad,'myGradxy.png');%图像生成
% imshow(Igrad);
end


