function  [coefficients,percet] = ransac(pointData,shapeType,deviation)
%Random Sample Consensus
% [coefficients,percet] = ransac(pointData,shapeType,deviation)
%
% INPUT:
% pointData - data to fit
% shapeType - type of fitting,1 for staight line,2 for quadratic curve,3 for
%             cubic curve,'plane' for plane fitting,'circle' for circle 
%             fitting,'ellipse' for ellipse fitting
% deviation - permited distance difference between inlier points and fitted
%             model
%
% OUTPUT:
% coefficients - fitted model's coefficients
% percet       - proportion of inlier points 
%
% References : Fischler, M.A. and Bolles, R.C. Random Sample Consensus: A
%              Paradigm for Model Fitting with Applications to Image
%              Analysis and Automated Cartography. Communications of the 
%              ACM , 24(6): 381–395, 1981.
%
% The program is written by Chen Qichao in his period of studying in master
% degree at Tongji University. You can redistribute or modify the program
% for non-commercial use. Any commercial use of this program is forbidden
% except being authorized.
%
% mail : mailboxchen@foxmail.com
% Copyright (C) 2015 - 2018  Tongji University

permitIterations = 100;
if ~exist('deviation','var')||isempty(deviation)||deviation==0
    warning('deviation is set to 0.3,it may not suitable for your data.');
    deviation = 0.3;
end

nPoint = size(pointData,1);

nSatisfiedPoint = 0;
mostSatisfiedPoint = 0;
iterations = 0;
coefficients = [];
while nSatisfiedPoint < nPoint*2/3 &&  iterations<permitIterations    %有2/3的数据符合拟合模型或达到最大迭代次数就可以退出了
    switch shapeType
        case 1
            [nSatisfiedPoint,coefficients]=  ransacline(pointData,deviation);%一次二维曲线
        case 2
            [nSatisfiedPoint,coefficients]=  ransaccurve2(pointData,deviation);%二次二维曲线
        case 3
            [nSatisfiedPoint,coefficients]=  ransaccurve3(pointData,deviation);%三次二维曲线
        case 'plane'
            [nSatisfiedPoint,coefficients]=  ransacplane(pointData,deviation);%平面拟合
        case 'circle'
            [nSatisfiedPoint,coefficients]=  ransaccircle(pointData,deviation);%平面拟合
        otherwise
            return;
    end
    if nSatisfiedPoint>mostSatisfiedPoint           %找到符合拟合直线数据最多的拟合直线
        mostSatisfiedPoint = nSatisfiedPoint;
        bestCoefficients=coefficients;          %找到最好的拟合直线
    end  
    iterations=iterations+1;
end
 percet = mostSatisfiedPoint/nPoint;%符合拟合参数的点比例
if mostSatisfiedPoint~=0
    coefficients = bestCoefficients;
else
    %迭代不收敛
    return;
end
% coefficients=polyfit(pointData(:,1),pointData(:,2),2); %普通最小而成拟合
% drawresult(pointData,shapeType,coefficients);
end
%----------------------------------------------
function drawresult(pointData,shapeType,coefficients)
%显示符合最佳拟合的数据
nPoint = size(pointData,1);
    if shapeType==1,
        for i=1:nPoint
            plot(pointData(i,1),pointData(i,2),'r.');hold on
        end
        %绘制拟合的直线
        x = [pointData(1,1) ;pointData(nPoint,1)];
        y = coefficients(1,1)*x+[coefficients(1,2);coefficients(1,2)];
        plot(x,y,'b-');
    elseif shapeType==2,
        for i=1:nPoint
            plot(pointData(i,1),pointData(i,2),'r.');hold on
        end
        %绘制二次曲线
        if 1 == size(coefficients,2),
            x= coefficients(1,1)*ones(nPoint,1);
            y = pointData(:,2);
        else
            x = pointData(:,1);
            y = coefficients(1,1).*x.^2+coefficients(1,2).*x+coefficients(1,3).*ones(nPoint,1);
        end
        plot(x,y,'b-');
    elseif strcmp(shapeType,'plane'),
        for i=1:nPoint
            plot3(pointData(i,1),pointData(i,2),pointData(i,3),'r.');hold on
        end
        %绘制平面
        if pointData(1,1)>pointData(nPoint,1),
            L = pointData(nPoint,1);R=pointData(1,1);
        else
            R =pointData(nPoint,1) ; L=pointData(1,1);
        end
        if pointData(1,2)>pointData(nPoint,2),
            yL = pointData(nPoint,2);yR=pointData(1,2);
        else
            yR =pointData(nPoint,2) ; yL=pointData(1,2);
        end
        [x,y]=meshgrid(L-5:1:R+5,yL-5:1:yR+5);
        a = coefficients(1,1);
        b = coefficients(2,1);
        c = coefficients(3,1);
        z=1/c-(a/c).*x-(b/c).*y;
        surf(x,y,z);
        axis equal;
    elseif strcmp(shapeType,'circle'),
        for i=1:nPoint
            plot(pointData(i,1),pointData(i,2),'r.');hold on
        end
        x0 = coefficients(1,1);
        y0 = coefficients(2,1);
        r = coefficients(3,1);
        pos = [x0-r y0-r 2*r 2*r];
        rectangle('Position',pos,'Curvature',[1 1],'EdgeColor','b');
        axis equal
    end
end

function [nSatisfiedPoint ,coefficients]=  ransacline(pointData,deviation)  
%line
    nPoint = size(pointData,1);
    nSatisfiedPoint = 0;
    SampIndex=floor(1+(nPoint-1)*rand(2,1));  %产生两个随机索引，找样本用，floor向下取整
    samp(1,:)=pointData(SampIndex(1),1:2);      %对原数据随机抽样两个样本
    samp(2,:)=pointData(SampIndex(2),1:2);
    x = samp(:,1);
    y = samp(:,2);
    if x(1,1)~=x(2,1),
        coefficients=polyfit(x,y,1);  
    elseif x(1,1)==x(2,1)&&y(1,1)~=y(2,1),
        %斜率无穷大时
        coefficients = x(1,1);
    else
        %两点坐标相同，求解条件不足
        nSatisfiedPoint = 0;
        coefficients = [];
        return;
    end
    p1 = samp(1,:);
    p2 = samp(2,:);
    dpp = norm(p2-p1);
    for i = 1:nPoint,
        p =  pointData(i,1:2);
        dist = abs(det([p2-p1;p-p1]))/dpp;
        if dist<deviation,
            nSatisfiedPoint = nSatisfiedPoint+1;
        end  
    end
end

function [nSatisfiedPoint ,coefficients]=  ransaccurve2(pointData,deviation)  
%quadratic curve
    nPoint = size(pointData,1);
    nSatisfiedPoint = 0;
    nRand = 0;
    randNum =  floor(1+(nPoint-1)*rand(1,1));
    SampIndex = -ones(3,1);
    while nRand<3,
        rand0 =  floor(1+(nPoint-1)*rand(1,1));
        if rand0~=SampIndex(1,1)&&rand0~=SampIndex(2,1)&&rand0~=SampIndex(3,1),
            nRand = nRand+1;
            SampIndex(nRand,1) = rand0;
        end
    end
    samp(1,:)=pointData(SampIndex(1),1:2);      %对原数据随机抽样两个样本
    samp(2,:)=pointData(SampIndex(2),1:2);
    samp(3,:)=pointData(SampIndex(3),1:2);
    x = samp(:,1);
    y = samp(:,2);
    if x(1,1)~=x(2,1)&&x(1,1)~=x(3,1)&&x(2,1)~=x(3,1),
        %三点不同
        coefficients=polyfit(x,y,1);
    elseif x(1,1)==x(2,1)&&x(1,1)==x(3,1)&&x(2,1)==x(3,1)&&y(1,1)~=y(2,1)&&y(1,1)~=y(3,1)&&y(2,1)~=y(3,1),
        %斜率无穷大时
        coefficients = x(1,1);
        dist = abs(pointData(:,1) - coefficients.*ones(nPoint,1));
        for i=1:nPoint,
            if dist(i)<deviation,
                nSatisfiedPoint = nSatisfiedPoint+1;
            end
        end
        return;
    else
        %至少存在两点坐标相同，求解条件不足
        nSatisfiedPoint = 0;
        coefficients = [];
        return;
    end
    
    coefficients=polyfit(x,y,2);    
    a = coefficients(1,1);%二次曲线的系数
    b = coefficients(1,2);
    c = coefficients(1,3);
    for iPoint = 1:nPoint,
        X = pointData(iPoint,1);
        Y = pointData(iPoint,2);
        coeffi = [2*a^2 3*a*b 2*a*c-2*a*Y+b^2+1 b*c-b*Y-X];%极值方程的系数
        root = roots(coeffi);
        minDist = -1;
        for i=1:3,
            x = root(i);
            if isreal(x),
                %在实数解中以最短距离为准
                x=real(x);%曲线上与（X，Y距离最近的点）
                y = a*x^2+b*x+c;
                dist = norm([X-x Y-y]);
                if minDist==-1,
                    minDist=dist;
                elseif dist<minDist,  
                    minDist = dist;
                end
            end
        end
        if minDist<deviation,
            nSatisfiedPoint = nSatisfiedPoint+1;
        end  
    end
end

function [nSatisfiedPoint,coefficients]=  ransacplane(pointData,deviation)  
%plane
    nPoint = size(pointData,1);
    nSatisfiedPoint = 0;
    nRand = 0;
    randNum =  floor(1+(nPoint-1)*rand(1,1));
    SampIndex = -ones(3,1);
    while nRand<3,
        rand0 =  floor(1+(nPoint-1)*rand(1,1));
        if rand0~=SampIndex(1,1)&&rand0~=SampIndex(2,1)&&rand0~=SampIndex(3,1),
            nRand = nRand+1;
            SampIndex(nRand,1) = rand0;
        end
    end
    samp(1,:)=pointData(SampIndex(1),1:3);      %对原数据随机抽样3个样本
    samp(2,:)=pointData(SampIndex(2),1:3);
    samp(3,:)=pointData(SampIndex(3),1:3);
    x = samp(:,1);
    y = samp(:,2);
    h = samp(:,3);
    A = samp;
    b = [1 1 1]';
    r = rank(A);
    if r==3,
        coefficients = A\b;
    else
      %条件不足，无法唯一确定平面  
      nSatisfiedPoint = 0;
      coefficients = [];
      return;
    end       
    a1 = coefficients(1,1);%平面方程的系数,Ax+By+Cz+1=0,其中[A B C] = [a1 a2 a3]
    a2 = coefficients(2,1);
    a3 = coefficients(3,1);
    a4 = -1;
    x0 = pointData(:,1);
    y0 = pointData(:,2);
    h0 = pointData(:,3);
    dist = abs(a1.*x0+a2.*y0+a3.*h0+a4)./norm([a1 a2 a3]);
    for i=1:nPoint,
        if dist(i)<deviation,
            nSatisfiedPoint = nSatisfiedPoint+1;
        end
    end
end

function [nSatisfiedPoint,coefficients] = ransaccircle(pointData,deviation)
    nPoint = size(pointData,1);
    nSatisfiedPoint = 0;
    coefficients = [];
    if(nPoint<3)
        return;
    end
    index = getsampleindex(nPoint,3);
    samp(1,:)=pointData(index(1),1:2);      %对原数据随机抽样3个样本
    samp(2,:)=pointData(index(2),1:2);
    samp(3,:)=pointData(index(3),1:2);
    x1 = samp(1,1);
    x2 = samp(2,1);
    x3 = samp(3,1);
    y1 = samp(1,2);
    y2 = samp(2,2);
    y3 = samp(3,2);
    a = x1-x2;
    b = y1-y2;
    c = x1-x3;
    d = y1-y3;
    e =((x1^2-x2^2)-(y2^2-y1^2))/2;
    f = ((x1^2-x3^2)-(y3^2-y1^2))/2;
    isCollineation = (det([a b;c d])==0);
    if ~isCollineation,
        x0 = -(d*e-b*f)/(b*c-a*d);
        y0 = -(a*f-c*e)/(b*c-a*d);
        r = norm([x0-x1,y0-y1]);
        coefficients = [x0 y0 r]';
    else
        return;
    end
    x = pointData(:,1);
    y = pointData(:,2);
    dist = abs(sqrt((x-x0).^2+(y-y0).^2)-r);
    for i=1:nPoint,
        if dist(i)<deviation,
            nSatisfiedPoint = nSatisfiedPoint+1;
        end
    end
end

function index = getsampleindex(nPoint,nSample)
%
%在1~nPoint中随机抽取nSample个不重复对像，返回对象索引号
    index = -ones(nSample,1);
    if (nSample>nPoint)
        return;
    end
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
        if isSave
            iSample = iSample+1;
            index(iSample,1) = rand0;
        end
    end
end




