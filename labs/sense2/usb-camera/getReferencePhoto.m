function [img] = getReferencePhoto(name, cam)
% GETREFERENCEPHOTO Captures reference photo for image segmentation.
%   bg_photo = getReferencePhoto() sets up camera and captures image 
%   bg_photo = getReferencePhoto(cam) captures image using passed in camera

%% Setup webcam if argument not passed in
if nargin == 1
    % camList = webcamlist
    cam = webcam(2);
    cam.ExposureMode = 'manual';
    cam.Exposure = -5;
    cam.WhiteBalanceMode = 'manual';
end
%% Take image
img = snapshot(cam);

%%  Save image
imwrite(img, name);

%% Clean Up
if nargin == 1
    % Once the connection is no longer needed, clear the associated variable.
    clear cam;
end