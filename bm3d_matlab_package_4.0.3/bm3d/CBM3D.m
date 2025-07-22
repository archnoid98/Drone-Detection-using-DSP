% Step 1: Set up paths and parameters
clc; clear; close all;

% Set BM3D path
bm3d_path = 'D:\DSP Project\bm3d_matlab_package_4.0.3\bm3d';
addpath(bm3d_path);

% Set input and output folders
inputFolder = 'Untitled-video-Made-with-Clipcha';
outputFolder = 'Denoised_Results_NEW';
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

% Create a log file
logFile = fopen(fullfile(outputFolder, 'processing_log.txt'), 'w');
fprintf(logFile, 'Denoising Log - %s\n\n', datestr(now));

% Step 2: Process each image from 0001.jpg to 0500.jpg
numImages = 500;
successCount = 0;
estimatedNoiseLevel = 10/255; % Conservative noise estimate

fprintf('Starting batch denoising of %d images...\n', numImages);
fprintf(logFile, 'Starting batch denoising of %d images...\n', numImages);

for i = 1:numImages
    filename = sprintf('%04d.jpg', i);
    inputPath = fullfile(inputFolder, filename);
    outputPath = fullfile(outputFolder, sprintf('denoised_%04d.png', i));
    
    fprintf('\nProcessing %s...\n', filename);
    fprintf(logFile, '\nProcessing %s...\n', filename);
    
    try
        % 1. Read and validate image
        if ~exist(inputPath, 'file')
            fprintf('File not found, skipping\n');
            fprintf(logFile, 'File not found, skipping\n');
            continue;
        end
        
        originalImage = imread(inputPath);
        originalImage = im2double(originalImage);
        
        % 2. Convert to grayscale while preserving color information
        if size(originalImage, 3) == 3
            fprintf('Converting to grayscale for denoising\n');
            fprintf(logFile, 'Converting to grayscale for denoising\n');
            
            % Extract luminance channel (convert to grayscale)
            grayImage = rgb2gray(originalImage);
            
            % Store color information
            colorInfo = originalImage ./ (grayImage + eps);
            colorInfo(isnan(colorInfo)) = 0;
            colorInfo(isinf(colorInfo)) = 0;
        else
            grayImage = originalImage;
            colorInfo = [];
        end
        
        % 3. Denoise the grayscale image
        tic;
        denoisedGray = BM3D(grayImage, estimatedNoiseLevel);
        processingTime = toc;
        
        % 4. Restore color information if original was color
        if ~isempty(colorInfo)
            fprintf('Restoring color information\n');
            fprintf(logFile, 'Restoring color information\n');
            
            % Apply color correction
            denoisedImage = denoisedGray .* colorInfo;
            
            % Clip values to [0,1] range
            denoisedImage = max(0, min(1, denoisedImage));
        else
            denoisedImage = denoisedGray;
        end
        
        % 5. Save the denoised image
        imwrite(denoisedImage, outputPath);  
        fprintf('Successfully saved denoised image\n');
        fprintf(logFile, 'Successfully saved denoised image\n');
        
        % 6. Calculate metrics
        metrics = struct();
        if exist('psnr', 'file')
            metrics.psnr = psnr(denoisedImage, originalImage);
        end
        if exist('ssim', 'file')
            metrics.ssim = ssim(denoisedImage, originalImage);
        end
        
        % 7. Log results
        fprintf('Processing time: %.2f sec\n', processingTime);
        fprintf('PSNR: %.2f dB, SSIM: %.4f\n', metrics.psnr, metrics.ssim);
        
        fprintf(logFile, 'Processing time: %.2f sec\n', processingTime);
        fprintf(logFile, 'PSNR: %.2f dB, SSIM: %.4f\n', metrics.psnr, metrics.ssim);
        fprintf(logFile, 'STATUS: SUCCESS\n');
        
        successCount = successCount + 1;
        
    catch ME
        fprintf('Error processing %s: %s\n', filename, ME.message);
        fprintf(logFile, 'ERROR: %s\n', ME.message);
        
        % Save original image for debugging
        debugPath = fullfile(outputFolder, ['error_' filename]);
        try
            imwrite(originalImage, debugPath);
            fprintf('Saved original image for debugging\n');
            fprintf(logFile, 'Saved original image for debugging\n');
        catch
            fprintf('Could not save debug image\n');
            fprintf(logFile, 'Could not save debug image\n');
        end
    end
    
    % Display progress
    if mod(i, 50) == 0
        fprintf('Progress: %d/%d (%.1f%%) - Success: %d\n', ...
            i, numImages, 100*i/numImages, successCount);
        fprintf(logFile, 'Progress: %d/%d (%.1f%%) - Success: %d\n', ...
            i, numImages, 100*i/numImages, successCount);
    end
end

% Close log file
fclose(logFile);

% Final report
fprintf('\n=== PROCESSING COMPLETE ===\n');
fprintf('Successfully processed: %d/%d images (%.1f%%)\n', ...
    successCount, numImages, 100*successCount/numImages);
fprintf('Log file saved to: %s\n', fullfile(outputFolder, 'processing_log.txt'));

% Display sample results
try
    % Find first and last processed images
    firstFile = '';
    lastFile = '';
    for i = 1:numImages
        testFile = sprintf('denoised_%04d.png', i);
        if exist(fullfile(outputFolder, testFile), 'file')
            if isempty(firstFile)
                firstFile = testFile;
            end
            lastFile = testFile;
        end
    end
    
    if ~isempty(firstFile)
        % Display first image comparison
        origFirst = imread(fullfile(inputFolder, sprintf('%04d.jpg', str2double(firstFile(10:13)))));
        denoisedFirst = imread(fullfile(outputFolder, firstFile));
        
        figure('Name', 'First Image Comparison');
        subplot(1,2,1); imshow(origFirst); title('Original First Image');
        subplot(1,2,2); imshow(denoisedFirst); title('Denoised First Image');
        
        % Display last image comparison
        origLast = imread(fullfile(inputFolder, sprintf('%04d.jpg', str2double(lastFile(10:13)))));
        denoisedLast = imread(fullfile(outputFolder, lastFile));
        
        figure('Name', 'Last Image Comparison');
        subplot(1,2,1); imshow(origLast); title('Original Last Image');
        subplot(1,2,2); imshow(denoisedLast); title('Denoised Last Image');
    end
catch
    fprintf('Could not display sample results\n');
end