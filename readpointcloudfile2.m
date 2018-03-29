function pointCloudData = readpointcloudfile2(pointCloudFilePath)
% read all of points from point cloud file
    fid=fopen(pointCloudFilePath,'r');
    tline=fgetl(fid);   %执行完后文件指针已经指向第二行
    lineByte = size(tline,2);
    %要多移动2位，可能是每一行数据开头结尾各占一位
    fseek(fid, -lineByte-2, 'cof');   
    lineData = regexp(tline, '\s+', 'split');
    col =  size(lineData,2);
    temp = col;
    for i = 1:temp,
        %除去首尾空格
        if strcmp(lineData{i},'');
            col = col-1;
        end
    end  
    if col==4,
        %xyzi
        data = fscanf(fid,'%f %f %f %f',[4,inf])';
        pointCloudData = data;
    elseif col==3,
        %xyz
        data = fscanf(fid,'%f %f %f',[3,inf])';
        pointCloudData = data(:,1:3);
    elseif col==7,
        %xyzirgb
        data = fscanf(fid,'%f %f %f %d %d %d %d',[7,inf])';
        pointCloudData = data(:,1:4);
    end
    fclose(fid);
end