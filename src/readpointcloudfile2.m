function pointCloudData = readpointcloudfile2(pointCloudFilePath,divisionStr)
% read all of points from point cloud text file
% pointCloudData = readpointcloudfile2(pointCloudFilePath,divisionStr)
% 
% pointCloudFilePath - path of pointcloud data
% divisionStr        - separate string,camma and space can be distinguished
%                      automaticlaly, the others must be assigned
%


% The program is written by Chen Qichao in his period of studying in master
% degree at Tongji University. You can redistribute or modify the program
% for non-commercial use. Any commercial use of this program is forbidden
% except being authorized.
%
% mail : mailboxchen@foxmail.com
% Copyright (C) 2015 - 2018  Tongji University

fid=fopen(pointCloudFilePath,'r');
tline=fgetl(fid);   %执行完后文件指针已经指向第二行
fclose(fid);

if ~exist('divisionStr','var')% comma or space separated
    if contains(tline,',')
        divisionType =',';
        divisionStr = ',';
    else
        divisionType ='s';
        divisionStr = ' ';
    end
elseif strcmp(divisionStr,' ') % space separated
    divisionType ='s';
    divisionStr = ' ';
else  % others
    divisionType = divisionStr;
end

    lineData = regexp(tline, strcat('\',divisionType,'+'), 'split');
    col =  size(lineData,2);
    temp = col;
    for i = 1:temp
        %除去首尾空格
        if strcmp(lineData{i},'')
            col = col-1;
        end
    end  
    fid=fopen(pointCloudFilePath,'r');
    if col==4
        %xyzi
        formatstr = strcat('%f',divisionStr,'%f',divisionStr,'%f',divisionStr,'%f');
        tmp = (textscan(fid,formatstr));
        data = [tmp{1} tmp{2} tmp{3} tmp{4}];
        pointCloudData = data;
    elseif col==3
        %xyz
        tmp = (textscan(fid,'%f %f %f'));
        data = [tmp{1} tmp{2} tmp{3}];
        pointCloudData = data(:,1:3);
    elseif col==7
        %xyzirgb
        tmp = (textscan(fid,'%f %f %f %d %d %d %d'));
        data = [tmp{1} tmp{2} tmp{3} tmp{4} tmp{5} tmp{6} tmp{7}];
        pointCloudData = data(:,1:4);
    end
    fclose(fid);
end