% Step 1: Set up paths and parameters
clc; clear; close all;

% Set BM3D path
bm3d_path = 'D:\DSP Project\bm3d_matlab_package_4.0.3\bm3d';
addpath(bm3d_path);

% Set input and output folders
inputFolder = 'Untitled-video-Made-with-Clipcha';
outputFolder = 'Denoised_Results';
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

% Create a detailed log file
logFile = fopen(fullfile(outputFolder, 'denoising_log.txt'), 'w');
fprintf(logFile, 'Batch Denoising Log - %s\n\n', datestr(now));

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
        % 1. File existence check
        if ~exist(inputPath, 'file')
            msg = sprintf('File not found: %s', filename);
            fprintf('%s\n', msg);
            fprintf(logFile, '%s\n', msg);
            continue;
        end
        
        % 2. Read image with additional checks
        try
            originalImage = imread(inputPath);
            fprintf('Image read successfully. Dimensions: %s\n', mat2str(size(originalImage)));
            fprintf(logFile, 'Image read successfully. Dimensions: %s\n', mat2str(size(originalImage)));
        catch readErr
            msg = sprintf('Error reading image: %s', readErr.message);
            fprintf('%s\n', msg);
            fprintf(logFile, '%s\n', msg);
            continue;
        end
        
        % 3. Convert to double and validate
        try
            originalImage = im2double(originalImage);
            if ~ismatrix(originalImage) && ~(ndims(originalImage) == 3 && size(originalImage,3) == 3)
                error('Invalid image dimensions');
            end
        catch convertErr
            msg = sprintf('Error converting image: %s', convertErr.message);
            fprintf('%s\n', msg);
            fprintf(logFile, '%s\n', msg);
            continue;
        end
        
        % 4. Denoising with detailed error handling
        tic;
        try
            if size(originalImage, 3) == 3
                fprintf('Processing as color image with CBM3D\n');
                fprintf(logFile, 'Processing as color image with CBM3D\n');
                denoisedImage = CBM3D(originalImage, estimatedNoiseLevel);
                methodUsed = 'CBM3D';
            else
                fprintf('Processing as grayscale image with BM3D\n');
                fprintf(logFile, 'Processing as grayscale image with BM3D\n');
                denoisedImage = BM3D(originalImage, estimatedNoiseLevel);
                methodUsed = 'BM3D';
            end
            processingTime = toc;
            
            % 5. Save the denoised image
            try
                imwrite(denoisedImage, outputPath);
                fprintf('Successfully saved denoised image\n');
                fprintf(logFile, 'Successfully saved denoised image\n');
                
                % 6. Calculate metrics
                metrics = calculate_metrics(originalImage, denoisedImage);
                log_metrics(filename, metrics, processingTime, logFile);
                
                successCount = successCount + 1;
                
            catch saveErr
                msg = sprintf('Error saving denoised image: %s', saveErr.message);
                fprintf('%s\n', msg);
                fprintf(logFile, '%s\n', msg);
            end
            
        catch denoiseErr
            processingTime = toc;
            msg = sprintf('Error during denoising: %s', denoiseErr.message);
            fprintf('%s\n', msg);
            fprintf(logFile, '%s\n', msg);
            
            % Try to save the original image if denoising failed
            try
                imwrite(originalImage, fullfile(outputFolder, ['failed_' filename]));
                fprintf('Saved original image for debugging\n');
                fprintf(logFile, 'Saved original image for debugging\n');
            catch
                fprintf('Could not save original image\n');
                fprintf(logFile, 'Could not save original image\n');
            end
        end
        
    catch outerErr
        msg = sprintf('Unexpected error: %s', outerErr.message);
        fprintf('%s\n', msg);
        fprintf(logFile, '%s\n', msg);
    end
    
    % Periodic progress report
    if mod(i, 10) == 0
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
fprintf('Detailed log saved to: %s\n', fullfile(outputFolder, 'denoising_log.txt'));

% Helper functions
function metrics = calculate_metrics(original, denoised)
    metrics = struct();
    try
        if exist('psnr', 'file')
            metrics.psnr = psnr(denoised, original);
        end
        if exist('ssim', 'file')
            metrics.ssim = ssim(denoised, original);
        end
    catch
        metrics.psnr = NaN;
        metrics.ssim = NaN;
    end
end

function log_metrics(filename, metrics, processingTime, logFile)
    fprintf('Processing time: %.2f sec\n', processingTime);
    fprintf('PSNR: %.2f dB, SSIM: %.4f\n', metrics.psnr, metrics.ssim);
    
    fprintf(logFile, 'Processing time: %.2f sec\n', processingTime);
    fprintf(logFile, 'PSNR: %.2f dB, SSIM: %.4f\n', metrics.psnr, metrics.ssim);
    fprintf(logFile, 'STATUS: SUCCESS\n');
end