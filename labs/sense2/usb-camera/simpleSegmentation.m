%% Read in image
img1 = imread('background.jpg');
img2 = imread('foreground.jpg');

%% Convert Images to Black and White
img1BW = rgb2gray(img1);
img2BW = rgb2gray(img2);

%% Subtract Images
imgDiff = abs(img2BW - img1BW);
% figure
% imshow(imgDiff)

%% Find Maximium Location of Difference
maxDiff = max(max(imgDiff));
[iRow,iCol] = find(imgDiff == maxDiff);
[m,n] = size(imgDiff);
% imshow(imgDiff)
% hold on
% plot(iCol,iRow,'r*')

%% Use imtool to Determine Threshold and Length
% imtool(imgDiff)

%% Threshhold Image
imgThresh = imgDiff > 60;
% figure
% imshow(imgThresh)
% hold on
% plot(iCol,iRow,'r*')
% hold off

%% Fill in Regions
imgFilled = bwareaopen(imgThresh, 15);
% figure
% imshow(imgFilled)

%% Overlay Onto Original Image
imgBoth = imoverlay(img2, imgFilled, [1 0 0]);
% figure
% imshow(imgBoth)
highlighted_img = imgBoth;