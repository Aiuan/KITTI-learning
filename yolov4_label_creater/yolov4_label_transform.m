clear all;close all;clc;

fileID = fopen('./train.txt');
train_id = textscan(fileID,'%s');
fclose(fileID);
train_id = train_id{1};

fileID = fopen('./val.txt');
val_id = textscan(fileID,'%s');
fclose(fileID);
val_id = val_id{1};

classes = struct('Car',0,'Van',1,'Truck',2,'Pedestrian',3,'Person_sitting',4,...
    'Cyclist',5,'Tram',6,'Misc',7,'DontCare',8);
%% kitti_train.txt
for i = 1:size(train_id,1)
    id = train_id{i};
    img_path = ['/home/aify/Desktop/kitti/object/training/image_2/',id,'.png'];
    
    label_path = ['F:\kitti\object\training\label_2\',id,'.txt'];
    fileID = fopen(label_path);
    label = textscan(fileID,'%s %.2f %u %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f');
    fclose(fileID);
    type = [label{1}];
    bbox = [label{5},label{6},label{7},label{8}];
    
    if i==1
        content = img_path;
    else
        content = strcat('\n',img_path);
    end    
    for obj = 1:size(type,1)
        content = strcat(content,32,num2str(bbox(obj,1),'%.2f'));%left
        content = strcat(content,',',num2str(bbox(obj,2),'%.2f'));%top
        content = strcat(content,',',num2str(bbox(obj,3),'%.2f'));%right
        content = strcat(content,',',num2str(bbox(obj,4),'%.2f'));%bottom
        content = strcat(content,',',num2str(classes.(type{obj}),'%u'));%class
    end
    
    fileID = fopen('kitti_train.txt','a');
    fprintf(fileID,content);
    fclose(fileID);  
end
%% kitti_val.txt
for i = 1:size(val_id,1)
    id = val_id{i};
    img_path = ['/home/aify/Desktop/kitti/object/training/image_2/',id,'.png'];
    
    label_path = ['F:\kitti\object\training\label_2\',id,'.txt'];
    fileID = fopen(label_path);
    label = textscan(fileID,'%s %.2f %u %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f');
    fclose(fileID);
    type = [label{1}];
    bbox = [label{5},label{6},label{7},label{8}];
    
    if i==1
        content = img_path;
    else
        content = strcat('\n',img_path);
    end    
    for obj = 1:size(type,1)
        content = strcat(content,32,num2str(bbox(obj,1),'%.2f'));%left
        content = strcat(content,',',num2str(bbox(obj,2),'%.2f'));%top
        content = strcat(content,',',num2str(bbox(obj,3),'%.2f'));%right
        content = strcat(content,',',num2str(bbox(obj,4),'%.2f'));%bottom
        content = strcat(content,',',num2str(classes.(type{obj}),'%u'));%class
    end
    
    fileID = fopen('kitti_val.txt','a');
    fprintf(fileID,content);
    fclose(fileID);  
end

%% Í³¼ÆÍ¼Æ¬³ß´ç
path = fullfile('F:\kitti\object\training\image_2');
dirOutput=dir(fullfile(path,'*.png'));
fileNames={dirOutput.name};

imgSize = zeros(size(fileNames,2),2);

for i=1:size(fileNames,2)
    img = imread(fullfile(path,fileNames{i}));
    imgSize(i,:) = size(img,[2,1]);
end
