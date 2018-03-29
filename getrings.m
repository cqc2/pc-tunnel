function [rings_locs,rings_num,rings_wdith] = getrings(pointCloudFilePath)
% 环缝识别
% [rings_locs,rings_num,rings_wdith] = getrings(pointCloudFilePath)
% pointCloudFilePath - 隧道点云文件路径
% rings_locs - 环缝里程位置
% rings_num - 环片数目
% rings_wdith - 环片宽度，二维列向量，第一列为环片宽度，第二列为标记量，0表示正常，1表示检测出的宽度存在异常
pxielSize = 0.01;
    [imageData,PCD] = getorthoimage(pointCloudFilePath,pxielSize,5,120,150);
    initialX = PCD(1,1);
    [rings_locs,rings_wdith,rings_num]=getringsfromimg(imageData);
    rings_locs = rings_locs*pxielSize+initialX;
    rings_wdith(:,2) = rings_wdith(:,2).*pxielSize;
    result = [rings_num;rings_locs];
    fid1=fopen('getrings_result.txt','wt');
    fprintf(fid1,'%.2f\n',result');
    fclose(fid1);
end