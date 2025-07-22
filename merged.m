clc; 
clear; 
close all;

% Parameters for Wiener filter
wiener_window_size = 7;  
half_win_wiener = floor(wiener_window_size / 2);

% Parameters for optical flow + temporal denoising
window_size = 15;          
half_window = floor(window_size/2);
step = 5;                 
scale = 20;                 
smooth_kernel_size = 5;    
pyramid_levels = 3;        

% Video Setup
videoFile = 'drone_video.mp4';  
vidReader = VideoReader(videoFile);

% Read first frame and preprocess
frame1_raw = readFrame(vidReader);
if size(frame1_raw,3) == 3
    frame1_gray = im2double(rgb2gray(frame1_raw));
else
    frame1_gray = im2double(frame1_raw);
end

% --- Wiener filter on first frame ---
diff = abs(frame1_gray - medfilt2(frame1_gray, [3 3]));
noise_var = (median(diff(:)) / 0.6745)^2;

padded_noisy = padarray(frame1_gray, [half_win_wiener half_win_wiener], 'symmetric');
kernel = ones(wiener_window_size) / (wiener_window_size^2);
local_mean = conv2(padded_noisy, kernel, 'valid'); % 'valid' reamins the same size after conv
local_mean_sq = conv2(padded_noisy.^2, kernel, 'valid');
local_var = local_mean_sq - local_mean.^2;
w = max(0, ((local_var - noise_var) ./ (local_var + eps)));
frame1 = local_mean + w .* (frame1_gray - local_mean);
frame1 = max(0, min(1, frame1)); % valid image intensity range of [0,1] for Grayscale img

pyramid1 = cell(pyramid_levels,1);
pyramid1{1} = frame1;
for l = 2:pyramid_levels
    pyramid1{l} = imresize(pyramid1{l-1}, 0.5);
end

% Create figure handles for separate windows
hFlowFig = figure('Name','Optical Flow with Quiver','NumberTitle','off');
hStep1Fig = figure('Name','Processing Steps Visualization 1','NumberTitle','off');
hStep2Fig = figure('Name','Processing Steps Visualization 2','NumberTitle','off');
hStep3Fig = figure('Name','Processing Steps Visualization 3','NumberTitle','off');
hStep4Fig = figure('Name','Processing Steps Visualization 4','NumberTitle','off');

while hasFrame(vidReader) 
    tic;
    
    % Read next frame and preprocess
    frame2_raw = readFrame(vidReader);
    if size(frame2_raw,3) == 3
        frame2_gray = im2double(rgb2gray(frame2_raw));
    else
        frame2_gray = im2double(frame2_raw);
    end
    
    % --- Wiener filter on current frame ---
    padded_noisy = padarray(frame2_gray, [half_win_wiener half_win_wiener], 'symmetric');
    local_mean = conv2(padded_noisy, kernel, 'valid');
    local_mean_sq = conv2(padded_noisy.^2, kernel, 'valid');
    local_var = local_mean_sq - local_mean.^2;
    w = max(0, ((local_var - noise_var) ./ (local_var + eps)));
    frame2 = local_mean + w .* (frame2_gray - local_mean);
    frame2 = max(0, min(1, frame2));
    
    % Build pyramid for second frame
    pyramid2 = cell(pyramid_levels,1);
    pyramid2{1} = frame2;
    for l = 2:pyramid_levels
        pyramid2{l} = imresize(pyramid2{l-1}, 0.5);
    end
    
    % Initialize flow at coarsest pyramid level
    u = zeros(size(pyramid1{pyramid_levels}));
    v = zeros(size(pyramid1{pyramid_levels}));
    
    for l = pyramid_levels:-1:1
        if l < pyramid_levels
            u = 2 * imresize(u, size(pyramid1{l})); %bilinear interpolation of Flow Value
            v = 2 * imresize(v, size(pyramid1{l}));
            [x, y] = meshgrid(1:size(pyramid1{l},2), 1:size(pyramid1{l},1));
            warped_im2 = interp2(pyramid2{l}, x + u, y + v, 'linear', 0); %bilinear interpolation of Image Pixel
        else
            warped_im2 = pyramid2{l};
        end
        
        kernel_x = 0.25*[-1 1; -1 1];
        kernel_y = 0.25*[-1 -1; 1 1];
        kernel_t = 0.25*ones(2);
        
        fx = conv2(0.5*(pyramid1{l} + warped_im2), kernel_x, 'same');
        fy = conv2(0.5*(pyramid1{l} + warped_im2), kernel_y, 'same');
        ft = conv2(pyramid1{l} - warped_im2, kernel_t, 'same');
        
        fx = padarray(fx, [half_window half_window], 'replicate');
        fy = padarray(fy, [half_window half_window], 'replicate');
        ft = padarray(ft, [half_window half_window], 'replicate');
        
        du = zeros(size(pyramid1{l}));
        dv = zeros(size(pyramid1{l}));
        
        for i = 1:size(pyramid1{l},1)
            for j = 1:size(pyramid1{l},2)
                fx_local = fx(i:i+2*half_window, j:j+2*half_window);
                fy_local = fy(i:i+2*half_window, j:j+2*half_window);
                ft_local = ft(i:i+2*half_window, j:j+2*half_window);
                
                fx_vec = fx_local(:);
                fy_vec = fy_local(:);
                ft_vec = ft_local(:);
                
                A = [fx_vec fy_vec];
                b = -ft_vec;
                
                if rank(A'*A) == 2
                    nu = (A'*A) \ (A'*b);
                    du(i,j) = nu(1);
                    dv(i,j) = nu(2);
                end
            end
        end
        
        u = u + du;
        v = v + dv;
    end
    
    % Smooth flow vector
    u = medfilt2(u, [smooth_kernel_size smooth_kernel_size]);
    v = medfilt2(v, [smooth_kernel_size smooth_kernel_size]);
    
    velocity_magnitude = sqrt(u.^2 + v.^2);
    velocity_direction = atan2(v, u);
    avg_velocity = mean(velocity_magnitude(:));
    avg_velocity_direction= mean(velocity_direction(:));
    
    % Motion-compensated temporal denoising
    [X, Y] = meshgrid(1:size(frame2,2), 1:size(frame2,1));
    im1_warped = interp2(frame1, X + u, Y + v, 'linear', 0);
    
    motion_mag_norm = velocity_magnitude / (max(velocity_magnitude(:)) + eps);
    weight_prev = exp(-8 * motion_mag_norm);
    tempDenoised = weight_prev .* im1_warped + (1 - weight_prev) .* frame2;
    
    % --- Upscale 2x with bicubic interpolation ---
    upscaled = imresize(tempDenoised, 2, 'bicubic');
    
    % --- Blind deconvolution with Gaussian PSF guess ---
    PSF = fspecial('gaussian', 7, 10); %  (7x7) Gaussian Point Spread Function (PSF) with a standard deviation of 10
    deblurred = deconvblind(upscaled, PSF, 10); % Number of iterations: 10
    
    % --- Histogram equalization color correction on NEXT frame ---
    if hasFrame(vidReader)
        lastFrameRaw = readFrame(vidReader);
        targetSize = size(deblurred(:,:,1));
        lastFrameResized = imresize(lastFrameRaw, targetSize(1:2));
        
        if size(lastFrameResized,3) == 3
            grayLastFrame = rgb2gray(lastFrameResized);
        else
            grayLastFrame = lastFrameResized;
        end
        grayLastFrame = im2double(grayLastFrame);
        equalizedGray = histeq(grayLastFrame);
        
        lastFrameDouble = im2double(lastFrameResized);

        correctedRGB = repmat(equalizedGray, [1 1 3]);
        %1: replicate along the rows 
        %1: replicate along the columns 
        %3: repeat the grayscale image 3 times along the color channels
        
        epsilon = 1e-6;
        grayLastFrameDouble = grayLastFrame + epsilon;
        
        for c = 1:3
            correctedRGB(:,:,c) = correctedRGB(:,:,c) .* (lastFrameDouble(:,:,c) ./ grayLastFrameDouble);
        end
        
        % --- Gamma correction for brightness adjustment ---
        gamma = 0.8; % (0.5-1.5 range)
        gammaCorrected = correctedRGB.^gamma;
        
        % --- Image sharpening ---
        sharpened = imsharpen(gammaCorrected,'Amount', 1.2,'Radius', 1.0,'Threshold', 0.1);  
        % 'Amount'-> strength of the sharpening
        % 'Radius' -> size of the region (blur radius)
        % 'Threshold' -> minimum contrast required for sharpening
        
    else
        sharpened = deblurred; % fallback if no next frame
    end
    
    % Calculate PSNR
    reference_frame = imresize(im2double(frame2_raw), size(deblurred(:,:,1)));
    if size(reference_frame, 3) == 3
        reference_gray = rgb2gray(reference_frame);
    else
        reference_gray = reference_frame;
    end
    
    mse_value = mean((reference_gray(:) - deblurred(:)).^2);
    psnr_value = 10 * log10(1 / (mse_value + eps));
    
    % === Figure 1: Optical flow with quiver on temporal denoised frame ===
    figure(hFlowFig);
    imshow(tempDenoised);
    hold on;
    quiver(X(1:step:end,1:step:end), Y(1:step:end,1:step:end),u(1:step:end,1:step:end)*scale, v(1:step:end,1:step:end)*scale,'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    hold off;
    title(sprintf('Optical Flow with Quiver\nAvg Velocity: %.4f < %.4frad px/frame', avg_velocity, avg_velocity_direction));
    
    % Set figure size for better visibility
    figWidth = 1200;
    figHeight = 500;
    
    % Step 1: Original vs Wiener Filtered
    figure(hStep1Fig);
    set(hStep1Fig, 'Position', [100, 100, figWidth, figHeight]);
    subplot(1,2,1);
    imshow(frame2_gray);
    title('Original Grayscale');
    subplot(1,2,2);
    imshow(frame2);
    title('Wiener Filtered');
    
    % Step 2: Temporal Denoised vs Upscaled
    figure(hStep2Fig);
    set(hStep2Fig, 'Position', [100, 100, figWidth, figHeight]);
    subplot(1,2,1);
    imshow(tempDenoised);
    title('Temporal Denoised');
    subplot(1,2,2);
    imshow(upscaled);
    title('Upscaled (2x Bicubic)');
    
    % Step 3: Blind Deconvolution vs Histogram Equalized
    figure(hStep3Fig);
    set(hStep3Fig, 'Position', [100, 100, figWidth, figHeight]);
    subplot(1,2,1);
    imshow(deblurred);
    title('Blind Deconvolution');
    subplot(1,2,2);
    imshow(correctedRGB);
    title('Histogram Equalized');
    
    % Step 4: Gamma Corrected vs Sharpened (Final Output)
    figure(hStep4Fig);
    set(hStep4Fig, 'Position', [100, 100, figWidth, figHeight]);
    subplot(1,2,1);
    imshow(gammaCorrected);
    title('Gamma Corrected');
    subplot(1,2,2);
    imshow(sharpened);
    title('Sharpened (Final Output)');
    drawnow;
    % Update previous frame and pyramid
    frame1 = frame2;
    pyramid1{1} = frame1;
    for l = 2:pyramid_levels
        pyramid1{l} = imresize(pyramid1{l-1}, 0.5);
    end
    
    fprintf('Frame processed in %.3f seconds, Avg Velocity: %.4f < %.4frad px/frame, PSNR: %.2f dB\n', toc, avg_velocity, avg_velocity_direction, psnr_value);
end