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