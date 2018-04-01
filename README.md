# pc-tunnel
This project is used for processing pointcould data of tunnel. It contains the most common methods which you can modify for individual needs. All code was tested in MATLAB2017a. <br>
Befour using the program， you should make sure that the input textfile or data is the format of X Y Z I，where  X Y Z is the coordinate and I is the intensity of pointcloud ，like this：<br>

0.0000 27455.0000 4.0600 139<br>
0.0000 27455.0000 4.0600 137<br>
0.0000 27455.0000 4.0600 138<br>
0.0100 27455.0000 4.0600 135<br>
0.0100 27455.0000 4.0500 129<br>
0.0100 27455.0000 4.0500 129<br>

The las file is also accepted,though the text file is preferable. If your data is the other types, you should transform the data first. Here list the main funciotns of project, datail specification can be seen in source code.

# The project includes:<br>
## 1. Gennerating orthoimage of tunnel wall
### [img,pcd] = getorthoimage(inputData)<br>
![](https://github.com/cqc2/pc-tunnel/blob/master/example/3D-pointcloud.png) 
![](https://github.com/cqc2/pc-tunnel/blob/master/example/orthoimage.png) 


## 2. Extracting longitude seam of tunnel 
### [rings_locs,rings_num,rings_wdith] = getrings(pointCloudFilePath)<br>
![](https://github.com/cqc2/pc-tunnel/blob/master/example/tunnel_joint_seam-longitude.png) 


## 3. Extracting latitude seam of tunnel  
### getinnerrings(pcdFilePathOrData)<br>
![](https://github.com/cqc2/pc-tunnel/blob/master/example/tunnel_joint_seam-latitude.jpg) 

# About
The program is written by Chen Qichao in his period of studying in master degree at Tongji University. You can redistribute or modify the program for non-commercial use. Any commercial use of this program is forbidden except being authorized.<br>

## Any question about this project, please feel free to contact me
mail: mailboxchen@foxmail.com <br>
Copyright (C) 2015 - 2018  Tongji University
