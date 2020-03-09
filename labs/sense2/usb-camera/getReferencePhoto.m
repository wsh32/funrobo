function [background_img] = getReferencePhoto(cam)
% GETREFERENCEPHOTO Captures reference photo for image segmentation.
%   bg_photo = getReferencePhoto() sets up camera and captures image 
%   bg_photo = getReferencePhoto(cam) captures image using passed in camera

%% Setup webcam if argument not passed in
if nargin == 0
    % camList = webcamlist
    cam = webcam(2);
    cam.ExposureMode = 'manual';
    cam.Exposure = -5;
    cam.WhiteBalanceMode = 'manual';
end
%% Take image
background_img = snapshot(cam);

%%  Save image
imwrite(background_img,'background.jpg');

%% Clean Up
if nargin == 0
    % Once the connection is no longer needed, clear the associated variable.
    clear cam;
end