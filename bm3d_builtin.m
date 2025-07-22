clc; clear; close all;

originalImage = imread('drone_bm3d.jpg');
originalImage = im2double(rgb2gray(originalImage));

%add Gaussian noise
noiseLevel = 25; % (0-255 scale)
noisyImage = imnoise(originalImage, 'gaussian', 0, (noiseLevel/255)^2);

%set up BM3D path
bm3d_path = 'D:\DSP Project\bm3d_matlab_package_4.0.3\bm3d';
addpath(bm3d_path);

%denoise based on image type
try
    % Grayscale image - use BM3D
    denoisedImage = BM3D(noisyImage, noiseLevel/255);
    fprintf('Using BM3D for grayscale image\n');
    figure;
    subplot(1,3,1); imshow(originalImage); title('Original Image');
    subplot(1,3,2); imshow(noisyImage); title('Noisy Image');
    subplot(1,3,3); imshow(denoisedImage); title('Denoised Image (BM3D)');
    
    if exist('psnr', 'file')
        psnrNoisy = psnr(noisyImage, originalImage);
        psnrDenoised = psnr(denoisedImage, originalImage);
        improvement = psnrDenoised - psnrNoisy;
        
        fprintf('\n=== PERFORMANCE METRICS ===\n');
        fprintf('PSNR - Noisy: %.2f dB\n', psnrNoisy);
        fprintf('PSNR - Denoised: %.2f dB\n', psnrDenoised);
        fprintf('PSNR Improvement: %.2f dB\n', improvement);
    end
    
catch ME
    fprintf('Error during BM3D processing:\n');
    fprintf('%s\n', ME.message);
end