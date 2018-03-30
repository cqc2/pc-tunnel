function A = LASreadAll(infilename)
% LASREADALL reads in all variables from a LAS 1.x data file (used with lidar data) 
%
% INPUT
% infilename: input file name  (for example, 'myinfile.las') 
%           
% OUTPUT
% A:    This is a structure containing all the data in the file.
%         See the file documentation for more information:
% http://www.asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html
% 
% EXAMPLE
% A = LASreadAll('infile.las')
%
% Cici Alexander
%     September 2008 (updated 26.09.2008)
% Amy Farris (afarris@usgs.gov) 
%      November 2013 Substatially altered to read in all variables from the file

% This file has the capability to load data files using any 1 of 5 formats.
% However, I was only able to test it with one of the formats. 
% I included support for the rest of the file formats b/c I
% thought someone might find it useful.  I sure hope they work!!!
% If the code crashes, it may be because either the pointDataFormatID 
% value was not what I expected, or the value for pointDataRecordLength 
% is not what I think it should be based on the format ID.  
% If pointDataRecordLength is different than I expected, then you may need 
% to change the values in fseek, eg: (c+????)
% The number added to c should be the sum of bytes of all the variables 
% that occur before the variable currently being read in.
%
% Also, the file I tested this code with was LAS format 1.2, I 
% think this code will run on later versions of LAS.
%
% A brief explanation of the LAS file format:
% LAS files are binary, they begin with header information.  Included in
% the header is the size of the header (in bytes); this is called
% 'OffsetToPointData', refered to a 'c' in this code.  After c bytes the
% data begins with the first x value, then the first y value and so on...
% (exactly what data is included depends on the file format).  Then the
% file continues with the second x value, the second y value and so on...  
% The header tells you how many bytes of data there are for each data point
% ('pointDataRecordLength', also refered to as 'p' in this code).
% So to read in all the x values, you start at the beginig of the file and
% skip c bytes:  "(fseek(fid, c, 'bof');"  
% Then you read in one value and skip p-4 bytes 
% (each x value consists of 4 bytes) and then read the next x and so on:
% "X1 = fread(fid,inf,'int32',p-4);"
%
% This is my (Amy Farris') first attempt at reading in a binary file.  
% I depended strongly on Cici's orignal file (LASRead.m) at first.  Most 
% of the code after about line # 134 was written by me, as are these
% garrolous beginning comments.  I hope they help.

%% Open the file
fid =fopen(infilename);

% Check whether the file is valid
if fid == -1
    error('Error opening file')
end

%% Read in important information from the header
% Check whether the LAS format is 1.1
fseek(fid, 24, 'bof');
VersionMajor = fread(fid,1,'uchar');
VersionMinor = fread(fid,1,'uchar');
% afarris2011Aug20 changed the following line to read LAS1.2 files
% if VersionMajor ~= 1 || VersionMinor ~= 1 
if VersionMajor ~= 1  
    error('LAS format is not 1.*')
end

% Read in the offset to point data
fseek(fid, 96, 'bof');
OffsetToPointData = fread(fid,1,'uint32');

% Read in the point data fotmat ID
fseek(fid, 104, 'bof');
pointDataFormatID = fread(fid,1,'uchar');

% Read in the point data record length
fseek(fid, 105, 'bof');
pointDataRecordLength = fread(fid,1,'short');

% Read in the scale factors and offsets required to calculate the coordinates
fseek(fid, 131, 'bof');
XScaleFactor = fread(fid,1,'double');
YScaleFactor = fread(fid,1,'double');
ZScaleFactor = fread(fid,1,'double');
XOffset = fread(fid,1,'double');
YOffset = fread(fid,1,'double');
ZOffset = fread(fid,1,'double');

% The number of bytes from the beginning of the file to the first point record
% data field is used to access the attributes of the point data
c = OffsetToPointData;

% The number of bytes to skip after reading in each value is based on
% 'pointDataRecordLength' So I need a short version of the variable name:
p = pointDataRecordLength;

%% Now read in the data

% Reads in the X coordinates of the points;  making use of the
% XScaleFactor and XOffset values in the header.
fseek(fid, c, 'bof');
X1 = fread(fid,inf,'int32',p-4);
A.x = X1*XScaleFactor+XOffset;

% Read in the Y coordinates of the points
fseek(fid, c+4, 'bof');
Y1 = fread(fid,inf,'int32',p-4);
A.y = Y1*YScaleFactor+YOffset;

% Read in the Z coordinates of the points
fseek(fid, c+8, 'bof');
Z1 = fread(fid,inf,'int32',p-4);
A.z = Z1*ZScaleFactor+ZOffset;

% Read in the Intensity values of the points
fseek(fid, c+12, 'bof');
A.intensity = fread(fid,inf,'uint16',p-2);

% Read in the Return Number of the points. The first return will have a
% return number of one, the second, two, etc.
% In the next two fread's, p needs to be in bits = 8*byte
fseek(fid, c+14, 'bof');
A.ReturnNumber = fread(fid,inf,'bit3',p*8-3);

% Read in the Number of Returns for a given pulse.
fseek(fid, c+14, 'bof');
fread(fid,1,'bit3');
A.NumberOfReturns = fread(fid,inf,'bit3',p*8-3);

% Read in classification
fseek(fid, c+15, 'bof');
A.classification = fread(fid,inf,'char',p-1);

% Read in scan angle Rank
fseek(fid, c+16, 'bof');
A.scanAngleRank = fread(fid,inf,'char',p-1);
A.scanAngleRankInfo = '-90 to 90 left side';

% Read in User ID
fseek(fid, c+17, 'bof');
A.userID = fread(fid,inf,'char',p-1);

% Read in Point SourceID
fseek(fid, c+18, 'bof');
A.pointSourceID = fread(fid,inf,'short',p-2);


%% Now read in data specific to certain file formats
% Remembeber, only the code that reads in format 1 was tested
switch pointDataFormatID
    case 1 
        if pointDataRecordLength ~= 28
            error('pointDataRecordLength is not what i expected')
        end
        % Read in 'Global Encoding', which tells us what the time variable is
        fseek(fid, 6, 'bof');
        globalEncoding = fread(fid,1,'short');
        if globalEncoding
            A.timeInfo = 'Time is "standard GPS time minus 1e9"';
            disp('Time is "standard GPS time minus 1e9"')
        else
            A.timeIinfo = 'Time is second of the GPS Week; and NO, we do not know WHICH week. ';
            disp('Time is second of the GPS Week; and NO, we do not know WHICH week. ')
        end
        
        % Read in time
        fseek(fid, c+20, 'bof');
        A.time = fread(fid,inf,'double',p-8);
    case 2
        if pointDataRecordLength ~= 26
            error('pointDataRecordLength is not what i expected')
        end
        % Read in color
        fseek(fid, c+20, 'bof');
        A.red = fread(fid,inf,'short',p-2);
        fseek(fid, c+22, 'bof');
        A.green = fread(fid,inf,'short',p-2);
        fseek(fid, c+24, 'bof');
        A.blue = fread(fid,inf,'short',p-2);
    case 3
        if pointDataRecordLength ~= 34
            error('pointDataRecordLength is not what i expected')
        end
        % Read in 'Global Encoding', which tells us what the time variable is
        fseek(fid, 6, 'bof');
        globalEncoding = fread(fid,1,'short');
        if globalEncoding
            A.timeInfo = 'Time is "standard GPS time minus 1e9"';
            disp('Time is "standard GPS time minus 1e9"')
        else
            A.timeInfo = 'Time is second of the GPS Week; and NO, we do not know WHICH week. ';
            disp('Time is second of the GPS Week; and NO, we do not know WHICH week. ')
        end
        
        % Read in time
        fseek(fid, c+20, 'bof');
        A.time = fread(fid,inf,'double',p-8);
        
        % Read in color
        fseek(fid, c+28, 'bof');
        A.red = fread(fid,inf,'short',p-2);
        fseek(fid, c+30, 'bof');
        A.green = fread(fid,inf,'short',p-2);
        fseek(fid, c+32, 'bof');
        A.blue = fread(fid,inf,'short',p-2);
    case 4
        if pointDataRecordLength ~= 57
            error('pointDataRecordLength is not what i expected')
        end
        % Read in 'Global Encoding', which tells us what the time variable is
        fseek(fid, 6, 'bof');
        globalEncoding = fread(fid,1,'short');
        if globalEncoding
            A.timeInfo = 'Time is "standard GPS time minus 1e9"';
            disp('Time is "standard GPS time minus 1e9"')
        else
            A.timeInfo = 'Time is second of the GPS Week; and NO, we do not know WHICH week. ';
            disp('Time is second of the GPS Week; and NO, we do not know WHICH week. ')
        end
        
        % Read in time
        fseek(fid, c+20, 'bof');
        A.time = fread(fid,inf,'double',p-8);
        
        fseek(fid, c+28, 'bof');
        A.wavePacketDescriptorIndex = fread(fid,inf,'char',p-1);
        fseek(fid, c+29, 'bof');
        % i am not sure that 'unit64' is correct below
        A.byteOffsettoWaveformData = fread(fid,inf,'unit64',p-8);
        fseek(fid, c+37, 'bof');
        A.waveformPacketSize = fread(fid,inf,'unit32',p-4);
        A.waveformPacketSizeInfo = 'in bytes';
        fseek(fid, c+41, 'bof');
        % i am not sure that 'float' is correct below
        A.returnPointWaveformLocation = fread(fid,inf,'float',p-4);
        fseek(fid, c+45, 'bof');
        A.Xt = fread(fid,inf,'float',p-4);
        fseek(fid, c+49, 'bof');
        A.Yt = fread(fid,inf,'float',p-4);
        fseek(fid, c+53, 'bof');
        A.Zt = fread(fid,inf,'float',p-4);
    case 5
        if pointDataRecordLength ~= 63
            error('pointDataRecordLength is not what i expected')
        end
        % Read in 'Global Encoding', which tells us what the time variable is
        fseek(fid, 6, 'bof');
        globalEncoding = fread(fid,1,'short');
        if globalEncoding
            A.timeInfo = 'Time is "standard GPS time minus 1e9"';
            disp('Time is "standard GPS time minus 1e9"')
        else
            A.timeInfo = 'Time is second of the GPS Week; and NO, we do not know WHICH week. ';
            disp('Time is second of the GPS Week; and NO, we do not know WHICH week. ')
        end
        
        % Read in time
        fseek(fid, c+20, 'bof');
        A.time = fread(fid,inf,'double',p-8);
        
        % Read in color
        fseek(fid, c+28, 'bof');
        A.red = fread(fid,inf,'short',p-2);
        fseek(fid, c+30, 'bof');
        A.green = fread(fid,inf,'short',p-2);
        fseek(fid, c+32, 'bof');
        A.blue = fread(fid,inf,'short',p-2);
        fseek(fid, c+34, 'bof');
        A.wavePacketDescriptorIndex = fread(fid,inf,'char',p-1);
        fseek(fid, c+35, 'bof');
        % i am not sure that 'unit64' is correct below
        A.byteOffsettoWaveformData = fread(fid,inf,'unit64',55);
        fseek(fid, c+43, 'bof');
        A.waveformPacketSize = fread(fid,inf,'unit32',p-4);
        A.waveformPacketSizeInfo = 'in bytes';
        fseek(fid, c+47, 'bof');
        % i am not sure that 'float' is correct below
        A.returnPointWaveformLocation = fread(fid,inf,'float',p-4);
        fseek(fid, c+51, 'bof');
        A.Xt = fread(fid,inf,'float',p-4);
        fseek(fid, c+53, 'bof');
        A.Yt = fread(fid,inf,'float',p-4);
        fseek(fid, c+57, 'bof');
        A.Zt = fread(fid,inf,'float',p-4);
end
