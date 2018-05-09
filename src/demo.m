tunnelData = readpointcloudfile2('../../LAS2.xyz');%读取隧道点云数据
[imageData,pointCloudData_ortho] = getorthoimage(tunnelData);%生成正射影像
imshow(imageData);
[rings_locs,rings_num,rings_wdith] = getrings(tunnelData);%纵接缝识别
imshow(imread('result_rings.png'));