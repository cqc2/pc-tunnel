function mytest(ScanLineArray)
% ScanLineArray = getscanline_faro(pointCloudData,2);%按轴变动提取扫描线,faro
nScanLine = size(ScanLineArray,2);
tmp = [];
Aseam = [];
outdata=  [];
for iScanLine = 1:nScanLine
    x = ScanLineArray(iScanLine).x;
    y = ScanLineArray(iScanLine).y;
    h = ScanLineArray(iScanLine).h;
    ins = ScanLineArray(iScanLine).ins;
    originalData = [x y h ins];
    
    %噪声滤除
    [x0,y0,r] = getcirclepara(ScanLineArray(iScanLine));%波动范围（-0.02~0.02）
    d = sqrt((y-x0).^2+(h-y0).^2)-r;
    data =  originalData(abs(d)<0.03,:);
   
   %点云按角度重排序
   px = data(:,1);
   py = data(:,2);
   ph = data(:,3);
   pins = data(:,4);
   A = atan2d(ph-y0,py-x0);%注意是atan2d(Y,X)
   A(A>=-90)=(A(A>=-90)+90);
   A(A<-90&A>=-180)=(A(A<-90&A>=-180)+450);
   [A_sorted, idx] = sortrows(A);
   data_sorted = data(idx,:);
   
   outdata = [outdata;data_sorted(A_sorted>97&A_sorted<253,:)];
   %误差曲线
   x2 = data_sorted(:,1);
   y2 = data_sorted(:,2);
   h2 = data_sorted(:,3);
   
   d2 = sqrt((y2-x0).^2+(h2-y0).^2)-r;
    figure(1);plot(A_sorted,d2,'r-');
    dd = smooth(d2,300);
    figure(2);hold on;plot(A_sorted,dd,'r-');
    curvedata = dd;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 顶点检测参数/peak detect parameters
    Fs = 1; %采样频率
    MinPeakProminence = max(curvedata)/1; %峰最小突起幅度门限
    threshold = 0; %峰值点与邻近点比较门限
    MinPeakHeight = max(curvedata)/5; %最小峰高度门限
    MinPeakDistance = 50/Fs; %最小峰间距门限，大概值
    nPeaks = 90; %最多找nPeaks个峰
    sortstr = 'none'; %结果排序
    Annotate = 'extents';
    %峰宽度计算标准,halfprom:半突起幅度宽； halfheight:半高宽
    WidthReference = 'halfprom';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % get MinPeakDistance
    [pks,locs,w,p] = ...
        findpeaks(curvedata,Fs,'MinPeakProminence',MinPeakProminence, ...
        'threshold',threshold,'MinPeakHeight',MinPeakHeight, ...
        'MinPeakDistance',MinPeakDistance,'npeaks',nPeaks, ...
        'sortstr',sortstr, ...
        'Annotate',Annotate,'WidthReference',WidthReference);
     figure(2);hold on;plot(A_sorted(locs),pks,'go');
       figure(3); plot3(x2,y2,h2,'g.');hold on;
       tmp = [tmp;x2(locs),y2(locs),h2(locs)];
Aseam = [Aseam;[A_sorted(locs) pks]];
end
% savepointcloud2file([outdata(:,2) outdata(:,1) outdata(:,3) outdata(:,4)],'22',0);
Aseam1 = Aseam(Aseam(:,1)<=180,:);
Aseam2 = Aseam(Aseam(:,1)>180,:);
[para ,percent]= ransac(Aseam1,0,2);
[para2 ,percent2]= ransac(Aseam2,0,2);
plot(Aseam1(:,1),Aseam1(:,2),'r.');
figure(2);plot(Aseam2(:,1),Aseam2(:,2),'r.');
figure(4);plot3(tmp(:,1),tmp(:,2),tmp(:,3),'r.','MarkerSize',5)
end





function [x0,y0,r] = getcirclepara(pointArray)
%
midIdx = ceil(size(pointArray,2)/2);
y = pointArray(midIdx).y;
h = pointArray(midIdx).h;
para = ransac([y h],'circle',0.02);
if(~isempty(para))
    x0 = para(1,1);%断面圆心坐标，对应点云中y、h
    y0 = para(2,1);
    r = para(3,1);
else
    x0 = 0;
    y0 = 0;
    r=0;
end
end