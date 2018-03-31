function savepointcloud2file(pointCloudData,fileName,isAppend)
    data = pointCloudData;
    if isAppend,
        fid1=fopen(strcat(fileName,'.xyz'),'at');
    else
        fid1=fopen(strcat(fileName,'.xyz'),'wt');
    end
    col = size(data,2);
    if col==3,
        %xyz
        fprintf(fid1,'%.4f %.4f %.4f\n',data');
    elseif col==4,
        %xyzi
        fprintf(fid1,'%.4f %.4f %.4f %d\n',data');
    elseif col==7,
        %xyzirgb
        fprintf(fid1,'%.4f %.4f %.4f %d %d %d %d\n',data');
    end
    fclose(fid1);
end