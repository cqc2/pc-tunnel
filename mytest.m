% datetime
% imageData = getorthoimage('LAS2.xyz',[],5);
% imshow(imageData);
% imwrite(imageData,strcat('80_110','.png'));%Í¼ÏñÉú³É
% datetime

% datetime
% I = imread('80_110.png');
% [rings_locs,rings_wdith,rings_num]=getringsfromimg(I);
% initialX = 0;
% pxielSize = 0.01;
% rings_locs = rings_locs*pxielSize+initialX;
%     rings_wdith(:,2) = rings_wdith(:,2).*pxielSize;
%     result = [rings_num;rings_locs];
%     fid1=fopen('getrings_result.txt','wt');
%     fprintf(fid1,'%.2f\n',result');
%     fclose(fid1);
% datetime

% [imageData,pointCloudData] = getorthoimage('rings/3.xyz',0.01,5,135,245,[],[],1);
I=imread('notuse/1.png');
gettopring(I);
% 
% hold off
datetime;
getinnerrings('LAS2.las');
datetime;











