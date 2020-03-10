%  Simple USB Camera Tutorial
%  This example shows how to use the snapshot function to
%  acquire live images from USB webcams.
%  MATLAB® Support Package for USB Webcams provides the ability to bring
%  live images from any USB video class (UVC) compliant webcam into MATLAB.
%  From https://www.mathworks.com/help/supportpkg/usbwebcams/examples/
%  acquiring-a-single-image-in-a-loop.html
%  Available webcam functions:
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
%% Example of Acquiring Multiple Frames
for idx = 1:5
    multiImg = snapshot(cam);
    image(multiImg);
end
%% Clean Up
% Once the connection is no longer needed, clear the associated variable.
clear cam
disp('SimpleUSBCameraTutorial Done')