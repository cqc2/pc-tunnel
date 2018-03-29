function [lines1,lines2,flag] = gettopring(I)
%
imshow(I);hold on;
flag = 0;
width = size(I,2);
lines1 = gettopringfromimg(I,-85,-75);
lines2 = gettopringfromimg(I,75,85);
v1 = lines1.point1-lines1.point2;
v2 = lines2.point1-lines2.point2;
k1 = v1(2)./v1(1);
k2 = v2(2)./v2(1);

%延长检测到的直线
lines1.point1(1,2) = lines1.point1(1,1)*k1+lines1.point1(1,2);
lines1.point1(1,1) = 0;
lines1.point2(1,2) = (width-lines1.point2(1,1))*k1+lines1.point2(1,2);
lines1.point2(1,1) = width;

lines2.point1(1,2) = lines2.point1(1,1)*k1+lines2.point1(1,2);
lines2.point1(1,1) = 0;
lines2.point2(1,2) = (width-lines2.point2(1,1))*k1+lines2.point2(1,2);
lines2.point2(1,1) = width;

% v1 = lines1.point1-lines1.point2;
% v2 = lines2.point1-lines2.point2;
% d1 = norm(v1);
% d2 = norm(v2);
% w1 = norm([lines1.point1-lines2.point1]);
% w2 = norm([lines1.point2-lines2.point2]);

showlines(lines1);
showlines(lines2);

end

function showlines(lines)
     for k=1:length(lines)
        xy=[lines(k).point1;lines(k).point2];   
        plot(xy(:,1),xy(:,2),'LineWidth',2);
     end
end

function lines = gettopringfromimg(I,sA,eA)
%
%图像预处理
Ivl = im2double(I);
Iedge=edge(Ivl,'canny',0.05);
Iedge = imdilate(Iedge,ones(2));

%霍夫直线检测
[H1,T1,R1] = hough(Iedge,'Theta',sA:eA);
Peaks=houghpeaks(H1,1);
lines=houghlines(Iedge,T1,R1,Peaks);
end




