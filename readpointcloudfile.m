function pointCloudData = readpointcloudfile(fid,nPoint)
% read specified number of points from point cloud file
% suitable for reading the large text file 
    tline=fgetl(fid);   %执行完后文件指针已经指向第二行
    lineByte = size(tline,2);
    %要多移动2位，可能是每一行数据开头结尾各占一位
    fseek(fid, -lineByte-2, 'cof');   
    lineData = regexp(tline, '\s+', 'split');
    col =  size(lineData,2);
    if col==4,
        %xyzi
        data = fscanf(fid,'%f %f %f %d',[4,nPoint])';
        pointCloudData = data;
    elseif col==7,
        %xyzirgb
        data = fscanf(fid,'%f %f %f %d %d %d %d',[7,nPoint])';
        pointCloudData = data(1:nPoint,1:4);
    end
    fseek(fid, 2, 'cof');  
end