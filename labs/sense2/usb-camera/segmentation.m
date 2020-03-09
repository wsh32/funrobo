function [highlighted_img] = segmentation(img1, img2)
% SEGMENTATION Performs image segmentation between foreground and
% background image

%% Convert Images to Black and White
% Background photo
img1BW = rgb2gray(img1);
% Foreground photo
img2BW = rgb2gray(img2);

%% Subtract Images
imgDiff = abs(img2BW - img1BW);

%% Find Maximium Location of Difference
maxDiff = max(max(imgDiff));
[iRow,iCol] = find(imgDiff == maxDiff);
[m,n] = size(imgDiff);

%% Threshhold Image
imgThresh = imgDiff > 60;

%% Fill in Regions
imgFilled = bwareaopen(imgThresh, 15);

%% Overlay Onto Original Image
imgBoth = imoverlay(img2, imgFilled, [1 0 0]);
highlighted_img = imgBoth;