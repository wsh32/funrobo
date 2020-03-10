% Color object tracking USB Camera Tutorial
% This example shows how to use the snapshot function to aquire
% live images from USB webcams, then find and track colored targets.
%
%  webcamlist- List of webcams connected to your system
%  webcam    - Connection to a webcam
%  preview    - Preview live video data from webcam
%  snapshot    - Acquire single image frame from a webcam
%  closePreview- Close webcam preview window
%  D. Barrett 2-16-20    Modified by (R. Gao, D. Tarazi)

%%
% Identifying Available Webcams
% The webcamlist function provides a cell array of webcams on the
% current system that MATLAB can access.
% webcam(2) is  {'Microsoft LifeCam Cinema'}
camList = webcamlist

%% Set up Connection to Webcam
cam = webcam(2)

%% Fix auto exposure set it to manual, set whitebalance to manual too
cam.ExposureMode = 'manual'
cam.Exposure = -5
cam.WhiteBalanceMode = 'manual'

%% Preview Video Stream
preview(cam)

%% Acquire a Frame
img = snapshot(cam);

%%  Display the frame in a figure window.
imtool(img);

%% Use Color Threshold APp to create a colorMask function (yellowMask)
% apply colorMask function to capture image
% this will create two new images BW is binary mask of image
% colorMaskImg will be the color image inside the mask
[colorMask, yellowImg] = yellowMask(img);
imtool(colorMask);

%% Preprocess image to remove noise
se = strel('disk', 50);
cleanImage = imopen(colorMask, se);
imtool(cleanImage);

%% Calculate the centoid of the white part of masked area (target)
targetCenter = regionprops(cleanImage, 'centroid');
% Store the x and y coordinates in a two column matrix
centroids = cat(1, targetCenter.Centroid);
% Display the original image with the centroid locations superimposed
imshow(img);
hold on
plot(centroids(:,1), centroids(:,2), 'b*')
text(centroids(:,1), centroids(:,2), ' Target Centroid')
hold off

%% Calculate the area of the target
targetArea = regionprops(cleanImage, 'area');
area = targetArea.Area;
% Store the x and y coordinates in a two column matrix
centroids = cat(1, targetCenter.Centroid);
% Display the binary image with the centroid locations superimposed
imshow(cleanImage)
hold on
plot(centroids(:,1), centroids(:,2), 'b*')
text(centroids(:,1), (centroids(:,2)+10), num2str(area))
hold off

%% Clean up
% Once the connection is no longer needed, clear the associated variable.
clear cam
disp('ColorShapeTrackerUSBCamera Done')