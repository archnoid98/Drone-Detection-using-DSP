clc;
clear all;
close all;

% --- Set video file path ---
videoFile = 'video_test.mp4';  % Replace with your actual video file path
vidReader = VideoReader(videoFile);

% --- Parameters ---
numFrames = 5;               % Number of frames to process for temporal averaging
targetSize = [240, 320];     % Resize frames to this size (height x width)

% --- Pre-allocate arrays ---
frames = zeros(targetSize(1), targetSize(2), numFrames, 'uint8');          % Grayscale denoised frames
originalFrames = zeros(targetSize(1), targetSize(2), 3, numFrames, 'uint8'); % Original RGB frames

disp('Capturing frames...');
k = 1;
while hasFrame(vidReader) && k <= numFrames
    frame = readFrame(vidReader);
    
    % Resize frame
    resizedFrame = imresize(frame, targetSize);
    
    % Convert to grayscale for denoising
    gray = rgb2gray(resizedFrame);
    
    % Simple spatial denoising: averaging filter (3x3 kernel)
    kernel = ones(3,3) / 9;
    denoised = conv2(double(gray), kernel, 'same');
    denoised = uint8(denoised);
    
    % Store frames
    frames(:,:,k) = denoised;
    originalFrames(:,:,:,k) = resizedFrame;
    
    k = k + 1;
end

disp('Processing...');

% --- Temporal denoise: average frames along time dimension ---
tempDenoised = uint8(mean(frames, 3));

% --- Super-resolution: upscale 2x with bicubic interpolation ---
upscaled = imresize(tempDenoised, 2, 'bicubic');

% --- Deblur: blind deconvolution using Gaussian PSF guess ---
PSF = fspecial('gaussian', 7, 10);  % Gaussian PSF kernel
deblurred = deconvblind(upscaled, PSF, 10);

% --- Color correction: histogram equalization on last frame ---
lastFrame = readFrame(vidReader);               % Next frame for color correction
lastFrame = imresize(lastFrame, targetSize);    % Resize to target size

% Grayscale luminance for histogram equalization
grayLastFrame = rgb2gray(lastFrame);
equalizedGray = histeq(grayLastFrame);

% Convert images to double for calculations
lastFrameDouble = double(lastFrame);
grayLastFrameDouble = double(grayLastFrame);

% Expand equalized grayscale to 3 channels
correctedRGB = repmat(equalizedGray, [1, 1, 3]);
correctedRGB = double(correctedRGB);

% Adjust RGB channels based on grayscale ratio
for c = 1:3
    correctedRGB(:,:,c) = correctedRGB(:,:,c) .* (lastFrameDouble(:,:,c) ./ grayLastFrameDouble);
end

% Gamma correction to adjust brightness
gamma = 1.2;
correctedRGB = correctedRGB .^ gamma;

% Convert back to uint8 for display
correctedRGB = uint8(correctedRGB);

% --- Sharpen the color-corrected image ---
sharpenedCorrectedRGB = imsharpen(correctedRGB, 'Radius', 2, 'Amount', 1);

% --- Visualization ---

figure;

% Original frame (Frame 1)
subplot(2,3,1), imshow(originalFrames(:,:,:,1)), title('Original Frame 1');

% Denoised frame (Frame 1)
subplot(2,3,2), imshow(frames(:,:,1)), title('Denoised Frame 1');

% Temporal averaged denoised frame
subplot(2,3,3), imshow(tempDenoised), title('Temporal Averaged Denoised');

% Upscaled frame
subplot(2,3,4), imshow(upscaled), title('Upscaled Bicubic');

% Deblurred frame
subplot(2,3,5), imshow(deblurred), title('Deblurred Frame 1');

% Color-corrected frame
subplot(2,3,6), imshow(correctedRGB), title('Color Corrected Frame 1');

sgtitle('Frame 1: Original to Processed');

% Show sharpened color-corrected frame in separate figure
figure;
imshow(sharpenedCorrectedRGB);
title('Sharpened Color Corrected Frame');
