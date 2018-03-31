function getring()
    pointCloudFilePath = 'las3.xyz';
    fid=fopen(pointCloudFilePath,'r');
    pointCloudData =  readpointcloudfile(fid,10000000);%读取指定个数点
    space=5;%
%       pointCloudData = readpointcloudfile2(pointCloudFilePath);%读取全部点
%       ScanLineArray = slice2scanlines(pointCloudData(1:space:end,:),1);%按点相邻点间距提取扫描线,sick
      ScanLineArray = getscanline_faro(pointCloudData(1:space:end,:),2);%按轴变动提取扫描线,faro
      samplePointArray = getsamplepoint(ScanLineArray,30,90,0,100);%样本区域
    pointCloudData = pointArray2Point(samplePointArray);
    pxielSize = 0.005;
% imageData = convertpointcloud2img(pointCloudData,pxielSize);
imageData = convertPD2img(pointCloudData,pxielSize);
I = imageData;
imshow(I);
imwrite(I,'dense_result.png');%图像生成
% I = imread('dense2.png');
% I = im2double(I);
[H1,H2] = gradienthist(I);
ringLine = getringline(H2);
nline = size(ringLine,2);
for i= 1:nline,
    I(:,abs(ceil(ringLine(i)))) = 1;
end 
imwrite(I,'dense_result.png');%图像生成
    a=0;
end

function  samplepoint = pointArray2Point(samplePointArray)
%
%将展开的图像存储到文件
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

function ringLine = getringline(H)
%
[maxHOrder,maxH,minHOrder,minH] = getcriticalline(H);
nH = size(maxHOrder,2);
%正负关键点差值，反应了图像中环片线的宽度，若果过大则说明图像中环片边界模糊，提取的结果可靠性较低
dmaxH_minH = maxHOrder-minHOrder;

%各类关键点的序列差值，理论上应该等于环片宽度，这里像素为0.5mm，所以1.2m宽的环
%片应该是240个像素，最后的输出结果中这个值应该接近0，或者趋近于同一个值（此时环
%片宽度存在系统误差，不等于1.2m，不是实际不等于1.2m而是由于点云x坐标系统误差导致
%由图像像素反算的宽度不等于1.2m）
ddmaxH = maxHOrder(2:end)-maxHOrder(1:end-1); 
ddminH = minHOrder(2:end)-minHOrder(1:end-1); 
%环片平均像素宽度
meanWidth = mean([ddminH(1:end-1) ddmaxH(1:end-1)]);
% meanWidth = 240;
HOrder = ceil((maxHOrder+minHOrder)/2);
ddH = HOrder(2:end)-HOrder(1:end-1)-meanWidth; 
%如果环片宽与环片均值只差小于2个像素，认为边界是比较准确的
base = [];
iBase = 0
for i =1:nH-1,
    if abs(ddH(1,i))<=2,
        iBase =iBase+1;
        base(iBase) = i;
        
    end
end
%对环片进行调整，类似于一维秩亏水准网平差，环片边界像素序号是高程，环片之间的像素个数差是高差
%
B = -eye(nH)+diag(ones(1,nH-1),1);
B = B(1:end-1,:);%系数矩阵
l = -ddH';
P = diag(1./abs(ddH));
Px = eye(nH)*0;
% for i=1:iBase,
%     Px(base(i),base(i)) = 1;
%     Px(base(i)+1,base(i)+1) = 1;
% % end
% Px(7,7) = 0;
% Px(12,12) = 0;
S = ones(nH,1);
W = B'*P*l;
N = B'*P*B;
Qp = inv(N+Px*S*S'*Px);
x = Qp*W;
HOrder_adjust = HOrder+x';
ringLine = HOrder;%环片分界线对应像素横坐标序号
a=0;
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
a=0;
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
    for iScanline = 1:nScanline,
        x = ScanLineArray(iScanline).x;
        y = ScanLineArray(iScanline).y;
        h = ScanLineArray(iScanline).h;
        ins = ScanLineArray(iScanline).ins;
%         nPoint = size(x,1);      
%         plot3(x*10,y*10,h*10,'r.');axis equal;hold on;
%         continue;
        if mod(iScanline,1000)||iScanline==1,
            para = ransac([y h],'circle',0.02);
            if(isempty(para))
                continue;
            end
            x0 = para(1,1);%断面圆心坐标，对应点云中y、h
            y0 = para(2,1);
            r = para(3,1);
            if iScanline~=1,
                lenght0 = norm([x0Pre-x0 y0Pre-y0]);
                lenght = lenght+lenght0;
            end
            x0Pre = x0;
            y0Pre = y0;
        end
        if lenght>=startL&&lenght<=endL,
            ;
        else
            continue;
        end 
            px = x;
            py = y;
            ph = h;
            pins = ins;
            A = atan2d(py-x0,ph-y0);
            A(A>=0)=(A(A>=0)+90);
            A(A<0&A>=-90)=(A(A<0&A>=-90)+90);
            A(A<-90&A>=-180)=(A(A<-90&A>=-180)+450);   
            [row,~]=find(A>=startA&A<=endA);
            samplePointArray(iScanline).x = px(row);
            samplePointArray(iScanline).y = py(row);
            samplePointArray(iScanline).h = ph(row);
            samplePointArray(iScanline).ins = pins(row);
            samplePointArray(iScanline).A = A(row);
            samplePointArray(iScanline).r = r;
            samplePointArray(iScanline).l = (A(row)./180).*pi*r;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% iSample=0;
%         for iPoint = 1:nPoint,
%             px = x(iPoint);
%             py = y(iPoint);
%             ph = h(iPoint);
%             pins = ins(iPoint);
%             A = atan2d(py-x0,ph-y0);
%             if A>=0,
%                 A = A+90;
%             elseif A<0&&A>=-90,
%                 A = A+90;
%             elseif A<-90&&A>=-180,
%                 A=A+450;
%             end         
%             if A>=startA&&A<=endA,
%                 iSample = iSample+1;
%                 samplePointArray(iScanline).x(iSample) = px;
%                 
%                 samplePointArray(iScanline).y(iSample) = py;
%                 samplePointArray(iScanline).h(iSample) = ph;
%                 samplePointArray(iScanline).ins(iSample) = pins;
%                 samplePointArray(iScanline).A(iSample) = A;
%                 samplePointArray(iScanline).r(iSample) = r;
%                 samplePointArray(iScanline).l(iSample) = (A/180)*pi*r;
%             end           
%         end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    end   
end

function  [maxTestHOrder,maxTestH,minTestHOrder,minTestH] = getcriticalline(H)
% 
% 
%检测环片分界线原则：统计梯度是左右0.7米内绝对值最大的（有正负两个极值）。
    nHist = size(H,2);%直方图是按x方向统计的，所以这里就是x方向的像素个数
    templetLength = 2*0.7/0.005;%换算成像素长度
%     ringLinePixelArray = zeros(1,ceil(nHist*0.005/1.2)+10);%
    stratHOrder = 1;
    endHOrder = templetLength;
    center = zeros(1,ceil(nHist*0.005/1.2)+10);
    if nHist>templetLength,
        testH = H(stratHOrder:endHOrder);%从testH中检测最值
        [sortedTestH,order]= sort(testH);
        minTestHOrder(1) = order(1);%负最值像素坐标
        maxTestHOrder(1) = order(endHOrder);
        minTestH(1) = sortedTestH(1);%负最值大小
        maxTestH(1) = sortedTestH(endHOrder);
        center(1) = ceil((minTestHOrder+maxTestHOrder)/2);%推算出的环片线大致位置
    else
        return;
    end   
    iRing = 1;  
    while endHOrder<nHist,
        iRing = iRing+1;
        preCenter = center(iRing-1);
        stratHOrder = preCenter+floor(0.5/0.005);
        endHOrder = stratHOrder+templetLength;
        if endHOrder>nHist,
            break;
%             endHOrder = nHist;
        end
        testH = H(stratHOrder:endHOrder);
        [sortedTestH order]= sort(testH);
        minTestHOrder(iRing) = stratHOrder+order(1)-1;
        maxTestHOrder(iRing) = stratHOrder+order(end)-1;
        minTestH(iRing) = sortedTestH(1);
        maxTestH(iRing) = sortedTestH(end);
        center(iRing) = ceil((minTestHOrder(iRing)+maxTestHOrder(iRing))/2);%推算出的环片线大致位置
    end

%     plot(1:22,minTestHOrder-maxTestHOrder);hold on;
%     plot(1:22,maxTestHOrder);
%     a=0;
end





