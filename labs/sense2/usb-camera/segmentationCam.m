function segmentationCam()
% Perform segmentation using Microsoft LifeCam camera

clear all; close all; clc; imaqreset;

%% Setup webcam
cam = webcam(2);
cam.ExposureMode = 'manual';
cam.Exposure = -5;
cam.WhiteBalanceMode = 'manual';

%  Create a handle to an imshow figure for faster updating
hShow = imshow(zeros(480,640,3,'uint8'));
title('Webcam Video');

%% Read in reference image
ref_vid_img = imread('background.jpg');

%% Perform segmentation and display to the screen
while isvalid(hShow)
    % Capture an image from the webcam
    vid_img = snapshot(cam);

    % Call the live segmentation function
    obj = segmentation(ref_vid_img, vid_img);

    %  Update the imshow handle with a new image 
    set(hShow, 'CData', obj);
    drawnow;
end

%% Clean up
clear cam;
